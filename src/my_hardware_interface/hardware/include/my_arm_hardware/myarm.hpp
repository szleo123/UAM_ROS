// Copyright 2020 ros2_control Development Team
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

#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <cstdint>
#include <limits>
#include <cmath>

#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace my_arm_hardware
{
class MyArmHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MyArmHardware);

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;
    
  /// Get the logger of the SystemInterface.
  rclcpp::Logger get_logger() const { return *logger_; }

  /// Get the clock of the SystemInterface.
  rclcpp::Clock::SharedPtr get_clock() const { return clock_; }

  bool wait_for_initial_feedback(std::array<double,6>& q, double timeout_sec);

private:
  // Parameters for the Serial communications 
  std::string serial_port_path_ {"/dev/ttyUSB0"};
  std::string reader_port_path_ {"/dev/ttyUSB1"};
  std::string gripper_port_path_ {"/dev/ttyUSB2"};
  unsigned int baudrate_ {115200};
  unsigned int gripper_baudrate_ {115200};
  uint8_t gripper_id_ = 1; 
  double pos_scale_ {1000.0}; // rad -> int16 scale
  double hw_slowdown_; // low-pass factor if no feedback 

  // Objects for logging
  std::shared_ptr<rclcpp::Logger> logger_;
  rclcpp::Clock::SharedPtr clock_;

  // Store the command for the simulated robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
  std::vector<double> hw_velocities_;
  
  // LibSerial for read() and write()
  LibSerial::SerialPort serial_;
  std::mutex serial_mtx_;
  bool serial_ok_ {false};

  LibSerial::SerialPort reader_;
  std::mutex reader_mtx_;
  bool reader_ok_ {false};

  // LibSerial for gripper 
  LibSerial::SerialPort gripper_;
  std::mutex gripper_mtx_;
  bool gripper_ok_ {false};

  // Used for reading data from the reader port
  std::vector<uint8_t> rx_buffer_;

  // Used for reading data from the gripper 
  std::vector<uint8_t> gripper_rx_;
  bool grip_waiting_ = false; 
  rclcpp::Time grip_deadline_; 
  double grip_last_send_ros_ = std::numeric_limits<double>::quiet_NaN();
  rclcpp::Time last_grip_tx_;

  // Used for prevent initial jumps in position when no feedback is available
  bool initial_positions_received_ = false;
  double initial_read_timeout_sec_ = 2.0; 
  rclcpp::Time last_feedback_time_; // optional for diagnostics

  // Helpers 
  static inline int16_t clamp_to_i16(double val)
  {
    if (val > std::numeric_limits<int16_t>::max()) return std::numeric_limits<int16_t>::max();
    if (val < std::numeric_limits<int16_t>::min()) return std::numeric_limits<int16_t>::min();
    return static_cast<int16_t>(val);
  }

  const double grip_ros_min_ = -0.69; 
  const double grip_ros_max_ = 0.0;
  inline uint16_t grip_to_units(double ros) const {
    
    double x = std::min(std::max(ros, this->grip_ros_min_), this->grip_ros_max_);
    double t = (x - this->grip_ros_min_) / (this->grip_ros_max_ - this->grip_ros_min_); // 0..1
    return static_cast<uint16_t>(std::lround(t * (1390.0 - 60.0)) + 60.0); // units range: 60..1390
  }

  inline double units_to_grip(int16_t units) const {
    double u = std::min(std::max((double)units, 60.0), 1390.0);
    double t = u / (1390.0 - 60.0); // 0..1
    return this->grip_ros_min_ + t * (this->grip_ros_max_ - this->grip_ros_min_);
  }
};

}  // namespace my_arm_hardware
