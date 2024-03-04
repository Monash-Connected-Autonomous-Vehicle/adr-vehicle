// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
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

#include <limits>
#include <vector>
#include <chrono>
#include <cmath>

#include "4wd_kit_hardware/4wd_kit_hardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace 4wd_kit_hardware
{
hardware_interface::CallbackReturn 4WDKitHardware::on_init(
        const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    4WDKitHardwareLoggerName = info_.name;

    if (info_.joints.size() != 1)
    {
      RCLCPP_FATAL_STREAM(
        rclcpp::get_logger(4WDKitHardwareLoggerName),
        "Hardware interface '" << info_.name << "got " << info_.joints.size() << " joints but expected 1");
      return CallbackReturn::ERROR;
    }

    for (const auto& interface : info_.joints[0].command_interfaces){
        if(!set_control_interface(interface, true)){
            return CallbackReturn::ERROR;
        }
    }

    for (const auto& interface : info_.joints[0].state_interfaces){
        if(!set_control_interface(interface, false)){
            return CallbackReturn::ERROR;
        }
    }

    control_mode_ = 4wd_kit_hardware::ControlMode::Undefined;

    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn 4WDKitHardware::on_configure(
        const rclcpp_lifecycle::State & previous_state)
{
    if (!mock_) {
    //get min_interval
        if (hw_velocity_.state.has_value()) {
            RCLCPP_INFO_STREAM(rclcpp::get_logger(4WDKitHardwareLoggerName),
                               "Getting min interval on BLCMD " << can_id_);
    }

    // check for resolver if there is a position interface
    if (hw_position_.state.has_value() || hw_position_.command.has_value()) {
        RCLCPP_INFO_STREAM(rclcpp::get_logger(4WDKitHardwareLoggerName),
                           "Checking for resolver on BLCMD " << can_id_);

        auto resolver_check = get_config<uint16_t>(BLCMDConfigCommand::HAS_RESOLVER);
        if (resolver_check.has_value()) {
            if (resolver_check.value()) {
                RCLCPP_INFO_STREAM(rclcpp::get_logger(4WDKitHardwareLoggerName),
                                   "Resolver detected on BLCMD " << can_id_);
                return CallbackReturn::SUCCESS;
            } else {
                RCLCPP_FATAL_STREAM(rclcpp::get_logger(4WDKitHardwareLoggerName),
                                    "No resolver detected on BLCMD " << can_id_);
                return CallbackReturn::ERROR;
            }
        }
        RCLCPP_FATAL_STREAM(rclcpp::get_logger(4WDKitHardwareLoggerName),
                            "Error with resolver request on BLCMD" << can_id_);
        return CallbackReturn::ERROR;

    }
    }
    
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface> 4WDKitHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  if (hw_position_.command.has_value()) {
      command_interfaces.emplace_back(
              info_.joints[0].name, hardware_interface::HW_IF_POSITION, &hw_position_.command.value());
  }
  if (hw_velocity_.command.has_value()) {
      command_interfaces.emplace_back(
              info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_.command.value());
  }
  if (hw_effort_.command.has_value()) {
      command_interfaces.emplace_back(
              info_.joints[0].name, hardware_interface::HW_IF_EFFORT, &hw_effort_.command.value());
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn 4WDKitHardware::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
{
    bus_->set_callbacks_enabled(true);
    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn 4WDKitHardware::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    bus_->set_callbacks_enabled(false);
    return CallbackReturn::SUCCESS;
}

hardware_interface::return_type 4WDKitHardware::read(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    bus_->spin();
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type 4WDKitHardware::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    switch(control_mode_){
        case 4wd_kit_hardware::ControlMode::Undefined:
            break;
        case 4wd_kit_hardware::ControlMode::Position:
            if (hw_position_.command.has_value()) {
//                RCLCPP_INFO_STREAM(rclcpp::get_logger(4WDKitHardwareLoggerName),
//                                   "Sending Position Command " << hw_position_.command.value());
                send_scaled<int16_t>(make_can_id(BLCMDSendCommand::DRIVE_POSITION),
                                     hw_position_.command.value() * reversed_multiplier_, hw_position_.max);
            } else {
                RCLCPP_FATAL(rclcpp::get_logger(4WDKitHardwareLoggerName), "No position command");
                return hardware_interface::return_type::ERROR;
            }
            break;
        case 4wd_kit_hardware::ControlMode::Velocity:
            if (hw_velocity_.command.has_value()) {
               RCLCPP_INFO_STREAM(rclcpp::get_logger(4WDKitHardwareLoggerName),
                                  "Sending velocity command " << hw_velocity_.command.value() * reversed_multiplier_);
                send_scaled<int16_t>(make_can_id(BLCMDSendCommand::DRIVE_VELOCITY),
                                     hw_velocity_.command.value() * reversed_multiplier_, hw_velocity_.max);
            } else {
                RCLCPP_FATAL(rclcpp::get_logger(4WDKitHardwareLoggerName), "No velocity command");
                return hardware_interface::return_type::ERROR;
            }
            break;
        case 4wd_kit_hardware::ControlMode::Effort:
            if (hw_effort_.command.has_value()) {
                send_scaled<int16_t>(make_can_id(BLCMDSendCommand::DRIVE_CURRENT),
                                     hw_effort_.command.value() * reversed_multiplier_, hw_effort_.max);
            } else {
                RCLCPP_FATAL(rclcpp::get_logger(4WDKitHardwareLoggerName), "No effort command");
                return hardware_interface::return_type::ERROR;
            }
            break;
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type
    4WDKitHardware::prepare_command_mode_switch(const std::vector<std::string> &start_interfaces,
                                               const std::vector<std::string> &stop_interfaces) {
   for (const auto& interface : stop_interfaces) {
       if (!stop_interface(interface)) {
           return hardware_interface::return_type::ERROR;
       }
   }

   for (const auto& interface : start_interfaces) {
       if (!start_interface(interface)) {
           return hardware_interface::return_type::ERROR;
       }
   }

    return hardware_interface::return_type::OK;
}


}  // namespace 4wd_kit_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  4wd_kit_hardware::4WDKitHardware, hardware_interface::SystemInterface)
