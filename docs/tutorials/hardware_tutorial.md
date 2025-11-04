# Writing of a Hardware Interface

In the case if the hardware interface is not available or not suited for the desired application, it can be developed in a custom way, what is the topic of the next section.

In ros2_control, hardware system components are integrated via user defined driver plugins that conform to the `HarwareInterface` public interface. Hardware plugins specified in the URDF are dynamically loaded during initialization using the `pluginlib` interface. More information about creating and using plugins can be found [here](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Pluginlib.html).

For the purpose of this tutorial, let's create the custom interface plugin `ScaraRobot` that will be used to simulate a scara robot. We want the simulated system to be controlled in joint position and provide information about its current position and velocity, as described in the ros2_control description [file](../scara_description/ros2_control/scara.ros2_control.urdf).  

To do so, in the `scara_hardware` package, let's first define the hardware plugin called `ScaraRobot` that inherits from  `hardware_interface::SystemInterface`. The `SystemInterface` is one of the offered hardware interfaces designed for a complete robot system. For example, The UR5 uses this interface. The `ScaraRobot` must implement five public methods:
1. `on_init`
2. `export_state_interfaces`
3. `export_command_interfaces`
4. `read`
5. `write`

These methods are defined in the [scara_robot.hpp](../scara_hardware/include/scara_hardware/scara_robot.hpp) header file as follows: 

```c++
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
#include "hardware_interface/types/hardware_interface_return_values.hpp"

class HARDWARE_INTERFACE_PUBLIC ScaraRobot : public hardware_interface::SystemInterface {
    public:
    CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;
    // private members
    // ...
}
```

## Initializing the hardware
Let's first have a look at the initialization `on_init` method. The `on_init` method is called once during ros2_control initialization if the `ScaraRobot` was specified in the URDF. This method should:
- Check the validity of the requested `command_interfaces` and `state_interfaces` w.r.t. the loaded driver
- Instantiate the communication with the robot hardware
- Allocate memory

Since in this tutorial the robot is simulated, no communication need to be established. Instead, vectors will be initialized that represent the state all the hardware using the initial values from the description file. The definition of this method is as follows: 

```c++
CallbackReturn ScaraRobot::on_init(const hardware_interface::HardwareInfo &info) {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    // allocate memory 
    hw_states_position_.resize(info_.joints.size() std::numeric_limits<double>::quiet_NaN());
    //...
    // check the validity of the description
    for (const hardware_interface::ComponentInfo & joint : info_.joints) {
        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
            return CallbackReturn::ERROR;
        }
    }
    // ...
    // initialize states
    for (uint i = 0; i < info_.joints.size(); i++) {
        hw_states_position_[i] = std::stod(info_.joints[i].state_interfaces[0].initial_value);
    }
    // ...
    return CallbackReturn::SUCCESS;
}
```
Note that the behavior of `on_init` is expected to vary depending on the URDF description file. The `SystemInterface::on_init(info)` call fills out the `info` object with specifics from the URDF. The `info` object has fields for joints, sensors, gpios, and more. This allows to check if the interface that is called is compatible with the description and use to parameters from the description file to set up the hardware. 

## Exporting interfaces

Next, `export_state_interfaces` and `export_command_interfaces` methods are called in succession. Their purpose is to create a handle to link the internal state/command variable with the ros2_control framework so that it can be accessed from any method. The `export_state_interfaces` method returns a vector of `StateInterface` describing the `state_interfaces` for each joint. The `StateInterface` objects are read only data handles that contain the interface name, interface type, and a pointer to a double data value of the internal state variable. For the `ScaraRobot`, the `export_state_interfaces` references `hw_states_position_` vector with the position state interface and is defined as follows: 
```c++
std::vector<hardware_interface::StateInterface> ScaraRobot::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
    }
    // ...
    return state_interfaces;
}
```
The `export_command_interfaces` method is nearly identical to the previous one. The difference is that a vector of `CommandInterface` is returned. The vector contains objects describing the `command_interfaces` for each joint. For the `ScaraRobot`, the `export_command_interfaces` references the `hw_commands_position_` vector with the position command interface and is defined as follows: 
```c++
std::vector<hardware_interface::CommandInterface> ScaraRobot::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++) {
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_position_[i]));
    }
    return command_interfaces;
}
```
Now that the Hardware Interface is initialized, connected to the robot and that the internal variables are connected to the ros2_control framework, let's focus on the main control loop. 

## Reading and writing from the hardware
In ros2_control the main control loop consists in successive calls of the hardware `read` method, followed by the controller `update` method, followed by the hardware `write` method. In the read phase of the main loop, ros2_control loops over all hardware components that where loaded to call their `read` method. It is executed on the realtime thread, hence the method must obey by realtime constraints. The `read` method is responsible for accessing the robot current state and updating the data values of the `state_interfaces`. 
In this tutorial, as we only want to simulate the robot, we compute its current velocity as follows:  
```c++
hardware_interface::return_type ScaraRobot::read(const rclcpp::Time & time, const rclcpp::Duration &period) {
    // read hardware values for state interfaces, e.g joint encoders and sensor readings
    for (uint i = 0; i < info_.joints.size(); i++) {
        hw_states_velocity_[i] = (hw_states_position_[i] - hw_states_previous_position_[i])/(period.nanoseconds()*1e-9);

        hw_states_previous_position_[i] = hw_states_position_[i];
    }
    return hardware_interface::return_type::OK;
}
```
In the same way, during the write phase of the main loop, the `write` method of all loaded hardware components is called after the controller `update` in the realtime loop. For this reason, the `write` method must also obey by realtime constraints. The `write` method is responsible for updating the data values of the `command_interfaces`. 
In the case of our scara robot, the methods is defined as follows:
```c++
hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) {
    // send command interface values to hardware, e.g joint set joint velocity
    bool isNan = false;
    for (auto i = 0ul; i < hw_commands_position_.size(); i++) {
        if (hw_commands_position_[i] != hw_commands_position_[i]) isNan = true;
    }

    if (!isNan) {
        for (uint i = 0; i < info_.joints.size(); i++) {
            double min_position = std::stod(info_.joints[i].state_interfaces[0].min);
            double max_position = std::stod(info_.joints[i].state_interfaces[0].max);

            hw_states_position_[i] = hw_commands_position_[i];

            if(hw_states_position_[i] > max_position) hw_states_position_[i] = max_position;
            if(hw_states_position_[i] < min_position) hw_states_position_[i] = min_position;
        }
    }

    return hardware_interface::return_type::OK;
}
```
Notice here that in the first part of the methods we check if the value command is valid (i.e. not NAN), that means that a command was received through the `command_interface`. Also, notice that we can use some information from the ros2_control description file such as position `min` and `max` parameters to prevent the robot to go outside of the its limits.

Now that we have defined our hardware interface, let's focus on building it in the next section.

## Building the Hardware Interface plugin
Building the Hardware Interface plugin is done with the following steps:
* Adding C++ export macro 
* Creating the plugin description file
* Exporting the CMake library

### Adding C++ export macro
In order to reference the previously defined hardware interface as a ros2_control plugin, we need to add the following two lines of code at the end of the [scara_robot.cpp](../scara_hardware/src/scara_robot.cpp) file containing our method definitions:

```c++
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(scara_hardware::ScaraRobot, hardware_interface::SystemInterface)
```
The `PLUGINLIB_EXPORT_CLASS` is a c++ macro that creates a plugin library using `pluginlib`. More information about it can be found [here](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Pluginlib.html).

### Creating the plugin description file
The plugin description file is a required XML file that describes the plugin's library name, class type, namespace, description, and interface type. This file allows ROS2 to automatically discover and load plugins. It is formatted as follows:

```xml
<library path="{Library_Name}">
  <class
    name="{Namespace}/{Class_Name}"
    type="{Namespace}::{Class_Name}"
    base_class_type="hardware_interface::SystemInterface">
  <description>
    {Human readable description}
  </description>
  </class>
</library>
```

The `path` attribute of the `library` tags refers to the cmake library name of the user defined hardware plugin. See [here](../scara_hardware/scara_hardware_plugin.xml) for the complete XML file used for this tutorial.

### Exporting the CMake library
The general CMake template to make a hardware plugin available in ros2_control is shown below:
```cmake
add_library(
    scara_hardware
    SHARED
    src/scara_robot.cpp
)

# include and link dependencies
# ...

# Causes the visibility macros to use dllexport rather than dllimport, which is appropriate when building the dll but not consuming it.
target_compile_definitions(scara_hardware PRIVATE "HARDWARE_PLUGIN_DLL")
# export plugin
pluginlib_export_plugin_description_file(scara_hardware scara_hardware_plugin.xml)
# install libraries
# ...
```
Notice that a library is created using the plugin source code just like any other cmake library. In addition, an extra compile definition and cmake export macro (`pluginlib_export_plugin_description_file`) need to be added. See [here](../scara_hardware/CMakeLists.txt) for the complete `CMakeLists.txt` file used for this tutorial. 

Now that our scara robot's hardware is ready to be loaded as a plugin let's run our scara robot!

To do so you just need to specify your hardware plugin in the ros2_control urdf description [file](../scara_description/ros2_control/scara.ros2_control.urdf) as follows:
```xml
<?xml version="1.0"?>
<robot name = "scara" xmlns:xacro="http://www.ros.org/wiki/xacro">
    <ros2_control name="scara" type="system">
        <hardware>
            <plugin>scara_hardware/ScaraRobot</plugin>
        </hardware>
    <!-- joints, gpios, sensors -->
    </ros2_control>
</robot>
```

Now you can test your hardware as explained in the previous [section on launching and interacting with the hardware](launch_tutorial.md), or got further and see the [section on how to develop a custom controller](controller_tutorial.md).