# Writing of a Controller

In ros2_control, controllers are implemented as plugins that conforms to the `ControllerInterface` public interface. Unlike hardware interfaces, controllers are [managed node](https://design.ros2.org/articles/node_lifecycle.html), which means that they work as state-machines and thus have a finite set of states, which are:

1. Unconfigured
2. Inactive
3. Active
4. Finalized

In order to properly manage controllers, certain interface methods are called when transitions between these states occur. During the main control loop, the controller needs to be in the active state. 
In the following development, we will focus on the requirements for writing a new controller interface. For the purpose of this tutorial, the developed controller will be a joint velocity controller taking as input an array of velocities and applying them to each position controlled joint. 

The controller plugin for the tutorial robot is called `ScaraJointVelocityController` that inherits from  `controller_interface::ControllerInterface`. The `ScaraJointVelocityController` must implement nine public methods. The latter 6 are [managed node](https://design.ros2.org/articles/node_lifecycle.html) transition callbacks.
1. `command_interface_configuration`
2. `state_interface_configuration`
3. `update`
4. `on_configure`
5. `on_activate`
6. `on_deactivate`
7. `on_cleanup`
8. `on_error`
9. `on_shutdown`

These methods are defined in the [scara_joint_velocity_controller.hpp](../scara_controllers/scara_joint_velocity_controller/include/scara_joint_velocity_controller/scara_joint_velocity_controller.hpp) header file as follows:
```c++
class ScaraJointVelocityController : public controller_interface::ControllerInterface {
    public:
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    CallbackReturn on_init() override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;
// private members
// ...
}
```
## Initializing the Controller
The `on_init` method is called immediately after the controller plugin is dynamically loaded. The method is called only once during the lifetime for the controller, hence memory that exists for the lifetime of the controller should be allocated. Additionally, the parameter values for `joints`, `command_interfaces` and `state_interfaces` should be declared and accessed. Those parameter values are required for the next two methods. 

In this tutorial, in the `on_init` method the `joints` parameter is declared as follows:
```c++
CallbackReturn ScaraJointVelocityController::on_init(){
    // declare and get parameters needed for controller initialization
    // allocate memory that will exist for the life of the controller
    // ...
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
    return CallbackReturn::SUCCESS;
}
```
This parameter allows to specify the joints that will be controlled with this controller.

## Configuring the Controller
The `on_configure` method is called immediately after the controller is set to the inactive state. This state occurs when the controller is started for the first time, but also when it is restarted. Reconfigurable parameters should be read in this method. Additionally, publishers and subscribers should be created.

In this tutorial, in this method the names of the controlled joints are queried and the subscription to the `~/joint_velocity` topic is made as follows:
```c++
CallbackReturn ScaraJointVelocityController::on_configure(const rclcpp_lifecycle::State &previous_state){
    // declare and get parameters needed for controller operations
    // setup realtime buffers, ROS publishers, and ROS subscribers

    joint_names_ = get_node()->get_parameter("joints").as_string_array();

    // the desired velocity is queried from the joint_velocity topic
    // and passed to update via a rt pipe
    joints_command_subscriber_ = get_node()->create_subscription<CmdType>(
        "~/joint_velocity", rclcpp::SystemDefaultsQoS(),
        [this](const CmdType::SharedPtr msg) {rt_command_ptr_.writeFromNonRT(msg);});
  return CallbackReturn::SUCCESS;
}
```
Notice here that the subscriber callback uses the `rt_command_ptr_` object to pass the received message. This step is very important as without it the subscriber would block the control loop that needs to be realtime. 

The `command_interface_configuration`  method is called after `on_configure`. The method returns a list of `InterfaceConfiguration` objects to indicate which command interfaces the controller needs to operate. The command interfaces are uniquely identified by their name and interface type. If a requested interface is not offered by a loaded hardware interface, then the controller will fail.

In our case, as we want the method to send commands to the position of the joints, the method is defined as follows:
```c++
controller_interface::InterfaceConfiguration ScaraJointVelocityController::command_interface_configuration(){
    controller_interface::InterfaceConfiguration conf;
    // add required command interface to `conf` by specifying their names and interface types.
    conf.names.reserve(joint_names_.size());
    for (const auto & joint_name : joint_names_) {
        conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    }
    return conf;
}
```

The `state_interface_configuration` method is then called, which is similar to the last method. The difference is that  a list of `InterfaceConfiguration` objects representing the required state interfaces to operate is returned.

Here again, for the purpose of this tutorial we only need the position state. The method is defined as follows: 
```c++
controller_interface::InterfaceConfiguration ScaraJointVelocityController::state_interface_configuration() {
    controller_interface::InterfaceConfiguration conf;
    // add required state interface to `conf` by specifying their names and interface types.
    conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    conf.names.reserve(joint_names_.size());
    for (const auto & joint_name : joint_names_) {
        conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    }    
    return conf;
}
```
## Activating the Controller
The `on_activate` is called once when the controller is activated. This method should handle controller restarts, such as setting the resetting reference to safe values. It should also perform controller specific safety checks. The `command_interface_configuration` and `state_interface_configuration` are also called again when the controller is activated.

In this tutorial, this method is used to order the command interfaces to fit the joint name order of the controller `joints` parameter. This is done by defining:   
```c++
CallbackReturn ScaraJointVelocityController::on_activate(const rclcpp_lifecycle::State &previous_state){
  // Handle controller restarts and dynamic parameter updating
  std::vector<std::reference_wrapper<LoanedCommandInterface>> ordered_interfaces;
  get_ordered_interfaces(
      command_interfaces_,
      joint_names_, 
      hardware_interface::HW_IF_POSITION, 
      ordered_interfaces)
  return CallbackReturn::SUCCESS;
}
```

## Running the Controller

The `update` method is part of the main control loop. Since the method is part of the realtime control loop, the realtime constraint must be enforced. The controller should read from the state interfaces, the reference and compute the command. Normally, the reference is access via a ROS2 subscriber. Since the subscriber runs on the non-realtime thread, a realtime buffer is used to a transfer the message to the realtime thread. The realtime buffer is eventually a pointer to a ROS message with a mutex that guarantees thread safety and that the realtime thread is never blocked. The calculated control command should then be written to the command interface, which will be passed to the hardware.

In this tutorial, the `update` method is defined as follows:
```c++
controller_interface::return_type ScaraJointVelocityController::update(const rclcpp::Time &time, const rclcpp::Duration &period){
    // Read controller inputs values from state interfaces
    // Calculate controller output values and write them to command interfaces

    // getting the data from the subscriber using the rt pipe
    auto joint_velocity = rt_command_ptr_.readFromRT();

    // no command received yet
    if (!joint_velocity || !(*joint_velocity)) {
        return controller_interface::return_type::OK;
    }

    // the states are given in the same order as defines in state_interface_configuration
    for(auto j = 0ul; j < joint_names_.size(); j++)
    {   
        double q = state_interfaces_[j].get_value();
        double vq = (*joint_velocity)->data[j];
    
        double command = q + vq*(period.nanoseconds()*1e-9);
        
        command_interfaces_[j].set_value(command);
    }

    return controller_interface::return_type::OK;
}
```
In this method, at first the data is queried from the subscriber using the `rt_command_ptr_` object. Then we check if a new command was received. Next, the controller iterates over all commanded joints and computes the commanded position to be applied and finally updates the `command_interfaces_` with the new position that needs to be passed to the hardware. 

## Additional methods
 The `on_deactivate` is called when a controller stops running. It is important to release the claimed command interface in this method, so other controllers can use them if needed. This is down with the `release_interfaces` function.

 In our case, to keep things simple this method is empty:
```c++
CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state){
    release_interfaces();
    // The controller should be properly shutdown during this
    // ...
    return CallbackReturn::SUCCESS;
}
```
The `on_cleanup` and `on_shutdown` are called when the controller's lifecycle node is transitioning to shutting down. Freeing any allocated memory and general cleanup should be done in these methods.
In our case, to keep things simple this methods are also empty:
```c++
CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state){
  // Callback function for cleanup transition
  // ...
  return CallbackReturn::SUCCESS;
}
```
```c++
CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state){
  // Callback function for shutdown transition
  // ...
  return CallbackReturn::SUCCESS;
}
```

The `on_error` method is called if the managed node fails a state transition. This should generally never happen.

In our case, to keep things simple this method is empty:
```c++
CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state){
  // Callback function for erroneous transition
  // ...
  return CallbackReturn::SUCCESS;
}
```
## Building the Controller plugin
Building the Controller plugin is done with the same steps as for the Hardware Interface:
* Adding C++ export macro 
* Creating the plugin description file
* Exporting the CMake library

### Adding C++ export macro
In order to reference the previously defined controller as a ros2_control plugin, we need to add the following two lines of code at the end of the [scara_joint_velocity_controller.cpp](../scara_controllers/scara_joint_velocity_controller/src/scara_joint_velocity_controller.cpp) file containing our method definitions:

```c++
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(scara_joint_velocity_controller::ScaraJointVelocityController, controller_interface::ControllerInterface)
```
The `PLUGINLIB_EXPORT_CLASS` is a c++ macro that creates a plugin library using `pluginlib`. More information about it can be found [here](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Pluginlib.html).

### Creating the plugin description file
The plugin description file is again required for the controller, since it is exported as a library. The controller plugin description file is formatted as follows:

```xml
<library path="{Library_Name}">
  <class
    name="{Namespace}/{Class_Name}"
    type="{Namespace}::{Class_Name}"
    base_class_type="controller_interface::ControllerInterface">
  <description>
    {Human readable description}
  </description>
  </class>
</library>
```
 See [here](../scara_controllers/scara_joint_velocity_controller/controller_plugin.xml) for the complete XML file.

### Exporting the CMake library
The plugin must be specified in the CMake file that builds the controller plugin. 

```cmake
add_library(
    scara_joint_velocity_controller
    SHARED
    src/scara_joint_velocity_controller.cpp
)

# include and link dependencies
# ...

# Causes the visibility macros to use dllexport rather than dllimport, which is appropriate when building the dll but not consuming it.
target_compile_definitions(scara_joint_velocity_controller PRIVATE "CONTROLLER_PLUGIN_DLL")
# export plugin
pluginlib_export_plugin_description_file(scara_joint_velocity_controller controller_plugin.xml)
# install libraries
# ...
```

See [here](../scara_controllers/scara_joint_velocity_controller/CMakeLists.txt) for the complete `CMakeLists.txt` file.

Now that the controller is ready to be used, let's add it to the [scara_controllers.yaml](../scara_description/config/scara_controllers.yaml) file and run it on the scara robot! 

To do so, in the [scara_controllers.yaml](../scara_description/config/scara_controllers.yaml) file add :
``` yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    # other controllers

    scara_joint_velocity_controller:
      type: scara_joint_velocity_controller/ScaraJointVelocityController

scara_joint_velocity_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2
      - joint3

# other controllers
```

You can now load and interact with the controller as explained in the previous [section](launch_tutorial.md).