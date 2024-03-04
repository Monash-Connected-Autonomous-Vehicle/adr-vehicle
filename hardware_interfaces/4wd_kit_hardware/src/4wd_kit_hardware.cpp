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

    if (info_.joints.size() != 1)
    {
      RCLCPP_FATAL_STREAM(
        rclcpp::get_logger(4WDKitHardwareLoggerName),
        "Hardware interface '" << info_.name << "got " << info_.gpios.size() << " joints but expected 1");
      return CallbackReturn::ERROR;
    }

    auto in1_search = info_.hardware_parameters.find("IN1");
    if (in1_search != info_.hardware_parameters.end()){
        in1_ = std::stol(pin_search->second);
    }

    auto in2_search = info_.hardware_parameters.find("IN2");
    if (in2_search != info_.hardware_parameters.end()){
        in2_ = std::stol(pin_search->second);
    }

    auto in3_search = info_.hardware_parameters.find("IN3");
    if (in3_search != info_.hardware_parameters.end()){
        in3_ = std::stol(pin_search->second);
    }

    auto in4_search = info_.hardware_parameters.find("IN4");
    if (in4_search != info_.hardware_parameters.end()){
        in4_ = std::stol(pin_search->second);
    }

    auto ena_search = info_.hardware_parameters.find("ENA");
    if (ena_search != info_.hardware_parameters.end()){
        ena_ = std::stol(pin_search->second);
    }

    auto enb_search = info_.hardware_parameters.find("ENB");
    if (enb_search != info_.hardware_parameters.end()){
        enb_ = std::stol(pin_search->second);
    }

    for (const auto& interface : info_.gpios[0].command_interfaces){
        if(!set_control_interface(interface, true)){
            return CallbackReturn::ERROR;
        }
    }

    /*
    for (const auto& interface : info_.gpios[0].state_interfaces){
        if(!set_control_interface(interface, false)){
            return CallbackReturn::ERROR;
        }
    }
    */

    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn 4WDKitHardware::on_configure(
        const rclcpp_lifecycle::State & previous_state)
{
    uint32_t left_offsets[2] = {IN1, IN2};
    uint32_t right_offsets[2] = {IN3, IN4};

    //Open GPIO lines
    chip = gpiod_chip_open_by_name(chipname);
    gpiod_chip_get_lines(chip, left_offsets, 2, &left);
    gpiod_chip_get_lines(chip, right_offsets, 2, &right);

    //Open lines for output
    gpiod_line_request_bulk_output(left, "4wd_kit_hardware", 0);
    gpiod_line_request_bulk_output(right, "4wd_kit_hardware", 0);
    
    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface> 4WDKitHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    if (hw_position_.command.has_value()) {
        command_interfaces.emplace_back(
                info_.gpios[0].name, hardware_interface::HW_IF_POSITION, &hw_position_.command.value());
    }
    if (hw_velocity_.command.has_value()) {
        command_interfaces.emplace_back(
                info_.gpios[0].name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_.command.value());
    }
    if (hw_effort_.command.has_value()) {
        command_interfaces.emplace_back(
                info_.gpios[0].name, hardware_interface::HW_IF_EFFORT, &hw_effort_.command.value());
    }

    return command_interfaces;
}

hardware_interface::CallbackReturn 4WDKitHardware::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
{
    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn 4WDKitHardware::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    return CallbackReturn::SUCCESS;
}

hardware_interface::return_type 4WDKitHardware::read(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type 4WDKitHardware::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

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
