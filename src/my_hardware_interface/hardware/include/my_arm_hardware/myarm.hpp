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

#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <libserial/SerialPort.h>
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_srvs/srv/trigger.hpp"

struct _XDisplay;
struct _XGC;
using Display = _XDisplay;
using Window = unsigned long;
using GC = _XGC *;

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
  enum class RosMasterHomingState : uint8_t
  {
    WAITING_INIT_COMMAND = 0,
    WAITING_DAMIAO_READY = 1,
    WAITING_OPERATOR_DROP_POSE = 2,
    WAITING_ALL_READY = 3,
    ALL_READY = 4,
  };

  // Parameters for the Serial communications 
  std::string serial_port_path_ {"/dev/ttyUSB0"};
  std::string reader_port_path_ {"/dev/ttyUSB1"};
  std::string gripper_port_path_ {"/dev/ttyUSB2"};
  unsigned int baudrate_ {115200};
  unsigned int gripper_baudrate_ {115200};
  uint8_t gripper_id_ = 1; 
  double pos_scale_ {1000.0}; // rad -> int16 scale
  std::array<double, 6> arm_joint_signs_ {{1.0, 1.0, -1.0, 1.0, 1.0, 1.0}};
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

  // Used for prevent initial jumps in position when no feedback is available
  bool initial_positions_received_ = false;
  double initial_read_timeout_sec_ = 2.0; 
  rclcpp::Time last_feedback_time_; // optional for diagnostics
  double feedback_stale_timeout_sec_ = 0.5;  // max allowed feedback age before writes stop
  bool first_power_on_ = true;

  // Helpers 
  static inline int16_t clamp_to_i16(double val)
  {
    if (val > std::numeric_limits<int16_t>::max()) return std::numeric_limits<int16_t>::max();
    if (val < std::numeric_limits<int16_t>::min()) return std::numeric_limits<int16_t>::min();
    return static_cast<int16_t>(val);
  }

  double aux_joint_min_ = -0.69;
  double aux_joint_max_ = 0.0;
  inline uint16_t grip_to_units(double ros) const {
    double lo = std::min(aux_joint_min_, aux_joint_max_);
    double hi = std::max(aux_joint_min_, aux_joint_max_);
    double x = std::min(std::max(ros, lo), hi);
    double span = hi - lo;
    if (span <= std::numeric_limits<double>::epsilon()) {
      return 60;
    }
    double t = (x - lo) / span; // 0..1
    return static_cast<uint16_t>(std::lround(t * (1390.0 - 60.0)) + 60.0); // units range: 60..1390
  }

  inline double units_to_grip(int16_t units) const {
    double u = std::min(std::max((double)units, 60.0), 1390.0);
    double t = (u - 60.0) / (1390.0 - 60.0); // 0..1
    return aux_joint_min_ + t * (aux_joint_max_ - aux_joint_min_);
  }

  // Optional realtime feedback UI
  bool feedback_plot_enabled_{false};
  double feedback_plot_rate_hz_{5.0};
  double feedback_plot_min_rad_{-3.14};
  double feedback_plot_max_rad_{3.14};
  std::vector<std::string> feedback_plot_joint_filter_;
  std::vector<size_t> feedback_plot_indices_;
  rclcpp::Time feedback_plot_last_render_{0, 0, RCL_ROS_TIME};
  size_t feedback_plot_history_length_{400};
  int feedback_plot_width_{640};
  int feedback_plot_height_{240};
  bool feedback_plot_windows_ready_{false};
  bool feedback_display_failed_{false};
  std::vector<std::string> feedback_plot_window_names_;
  std::vector<std::vector<double>> feedback_plot_history_;
  std::vector<std::vector<double>> feedback_plot_command_history_;
  std::vector<double> last_sent_commands_;
  Display *feedback_display_{nullptr};
  int feedback_screen_{0};
  unsigned long feedback_color_bg_{0};
  unsigned long feedback_color_grid_{0};
  unsigned long feedback_color_line_{0};
  unsigned long feedback_color_cmd_line_{0};
  unsigned long feedback_color_text_{0};
  std::vector<Window> feedback_windows_;
  std::vector<GC> feedback_window_gcs_;
  bool feedback_plot_csv_enabled_{false};
  bool feedback_plot_csv_append_{false};
  std::string feedback_plot_csv_path_{"joint_feedback_log.csv"};
  std::ofstream feedback_plot_csv_stream_;
  std::mutex feedback_plot_csv_mutex_;

  void update_feedback_plot_selection();
  void ensure_feedback_plot_storage();
  void ensure_feedback_windows_created();
  void maybe_render_feedback_plot();
  void render_joint_plot(size_t slot, size_t joint_idx);
  void ensure_feedback_csv_ready();
  void log_feedback_csv(size_t joint_idx, double stamp_sec, double feedback, double command);
  void close_feedback_csv();
  void reset_joint_buffers(double value);
  void sync_commands_to_states();
  size_t arm_joint_count() const;
  bool has_aux_joint() const;
  size_t aux_joint_index() const;
  void setup_homing_interface();
  void teardown_homing_interface();
  void publish_homing_state(RosMasterHomingState state, const std::string & message);
  void handle_system_event(uint8_t event_code);
  bool send_system_command(uint8_t command_code, std::string & failure_reason);
  bool start_damiao_initialization(const std::string & source, std::string & message);
  bool confirm_drop_pose(const std::string & source, std::string & message);
  bool can_write_motion_commands();
  void capture_post_homing_command_hold();
  void finish_post_homing_command_hold();
  void enforce_post_homing_command_hold();

  std::shared_ptr<rclcpp::Node> homing_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> homing_executor_;
  std::thread homing_spin_thread_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr homing_status_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr homing_state_pub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr homing_init_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr homing_drop_pose_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr homing_init_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr homing_confirm_srv_;
  std::mutex homing_state_mtx_;
  RosMasterHomingState homing_state_{RosMasterHomingState::WAITING_INIT_COMMAND};
  bool sync_commands_on_next_feedback_{false};
  std::mutex command_hold_mtx_;
  std::vector<double> post_homing_hold_commands_;
  bool post_homing_command_hold_active_{false};
};

}  // namespace my_arm_hardware
