// Copyright 2022, ICube Laboratory, University of Strasbourg
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "scara_cartesian_velocity_controller/scaraCartesianVelocityController.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace scara_cartesian_velocity_controller
{
using hardware_interface::LoanedCommandInterface;

ScaraCartesianVelocityController::ScaraCartesianVelocityController()
: controller_interface::ControllerInterface(),
  rt_command_ptr_(nullptr),
  joints_command_subscriber_(nullptr)
{
}

CallbackReturn ScaraCartesianVelocityController::on_init()
{
  try {
    // definition of the parameters that need to be queried from the
    // controller configuration file with default values
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn ScaraCartesianVelocityController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // getting the names of the joints to be controlled
  joint_names_ = get_node()->get_parameter("joints").as_string_array();

  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter was empty");
    return CallbackReturn::FAILURE;
  }
 
  // the desired cartesian velocity is queried from the cartesian_velocity topic
  // and passed to update via a rt pipe
  joints_command_subscriber_ = get_node()->create_subscription<CmdType>(
    "~/cartesian_velocity", rclcpp::SystemDefaultsQoS(),
    [this](const CmdType::SharedPtr msg) {rt_command_ptr_.writeFromNonRT(msg);});

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}
// As the demo scara hardware interface only supports position commands, it can be directly
// defined here without the need of getting as parameter. The position interface is then 
// affected to all controlled joints.
controller_interface::InterfaceConfiguration
ScaraCartesianVelocityController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(joint_names_.size());
  for (const auto & joint_name : joint_names_) {
    conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
  }
  return conf;
}
// The controller requires the current position states. For this reason
// it can be directly defined here without the need of getting as parameters.
// The state interface is then deployed to all targeted joints.
controller_interface::InterfaceConfiguration
ScaraCartesianVelocityController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(joint_names_.size());
  for (const auto & joint_name : joint_names_) {
    conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
  }
  return conf;
}

// Fill ordered_interfaces with references to the matching interfaces
// in the same order as in joint_names
template<typename T>
bool get_ordered_interfaces(
  std::vector<T> & unordered_interfaces, const std::vector<std::string> & joint_names,
  const std::string & interface_type, std::vector<std::reference_wrapper<T>> & ordered_interfaces)
{
  for (const auto & joint_name : joint_names) {
    for (auto & command_interface : unordered_interfaces) {
      if (command_interface.get_name() == joint_name + "/" + interface_type) {
        ordered_interfaces.push_back(std::ref(command_interface));
      }
    }
  }

  return joint_names.size() == ordered_interfaces.size();
}

CallbackReturn ScaraCartesianVelocityController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::vector<std::reference_wrapper<LoanedCommandInterface>> ordered_interfaces;
  if (
    !get_ordered_interfaces(
      command_interfaces_, joint_names_, hardware_interface::HW_IF_POSITION, ordered_interfaces) ||
    command_interfaces_.size() != ordered_interfaces.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %zu position command interfaces, got %zu",
      joint_names_.size(),
      ordered_interfaces.size());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn ScaraCartesianVelocityController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}
// main control loop function getting the state interface and writing to the command interface
controller_interface::return_type ScaraCartesianVelocityController::update(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & period)
{
  // getting the data from the subscriber using the rt pipe
  auto cartesian_velocity = rt_command_ptr_.readFromRT();

  // no command received yet
  if (!cartesian_velocity || !(*cartesian_velocity)) {
    return controller_interface::return_type::OK;
  }

  //checking proxy data validity
  if ((*cartesian_velocity)->data.size() != joint_names_.size())
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(), 1000, "command size does not match number of interfaces");
    return controller_interface::return_type::ERROR;
  }

    // the states are given in the same order as defines in state_interface_configuration
    Eigen::Vector3d q;
    q << state_interfaces_[0].get_value(),
        state_interfaces_[1].get_value(),
        state_interfaces_[2].get_value();

    double l1 = 0.425;
    double l2 = 0.345;

    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(3,3);
    J(0,0) = -l1*sin(q(0))-l2*sin(q(0)+q(1));
    J(0,1) = -l2*sin(q(0)+q(1));
    J(1,0) = l1*cos(q(0))+l2*cos(q(0)+q(1));
    J(1,1) = l2*cos(q(0)+q(1));
    J(2,2) = -1;
    
    Eigen::Vector3d v;
    v << (*cartesian_velocity)->data[0], 
        (*cartesian_velocity)->data[1], 
        (*cartesian_velocity)->data[2];

    Eigen::Vector3d vq = J.inverse()*v;
    
    Eigen::Vector3d command = q + vq*(period.nanoseconds()*1e-9);

    if(command(0) > 1.57) command(0) = 1.57;
    if(command(0) < -1.57) command(0) = -1.57;
    if(command(1) > 1.57) command(1) = 1.57;
    if(command(1) < -1.57) command(1) = -1.57;
    if(command(2) > 0.3) command(2) = 0.3;
    if(command(2) < 0) command(2) = 0;

    command_interfaces_[0].set_value(command(0));
    command_interfaces_[1].set_value(command(1));
    command_interfaces_[2].set_value(command(2));

  return controller_interface::return_type::OK;
}

}  // namespace scara_cartesian_velocity_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  scara_cartesian_velocity_controller::ScaraCartesianVelocityController, controller_interface::ControllerInterface)