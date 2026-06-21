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
#include <atomic>
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
#include <dynamixel_sdk/dynamixel_sdk.h>
#ifdef MY_ARM_HARDWARE_HAS_PINOCCHIO
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#endif
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "my_arm_hardware/srv/get_gripper_protection.hpp"
#include "my_arm_hardware/srv/gripper_status.hpp"
#include "my_arm_hardware/srv/set_gripper_protection.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_srvs/srv/trigger.hpp"

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

  enum class ArmCommandFrameFormat : uint8_t
  {
    LEGACY_POSITION = 0,
    POSITION_TORQUE = 1,
  };

  enum class DynamicsMode : uint8_t
  {
    GRAVITY = 0,
    CORIOLIS = 1,
    FULL = 2,
  };

  enum class Stm32ControlMode : uint8_t
  {
    POSITION_ONLY = 0,
    POSITION_TORQUE = 1,
    FULL_MIT = 2,
  };

  enum class GripperBackend : uint8_t
  {
    LINEAR_ACTUATOR = 0,
    DYNAMIXEL_XW430 = 1,
  };

public:
  struct GripperStatusData
  {
    bool valid{false};
    int target_units{0};
    int position_units{0};
    double position{0.0};
    int current_ma{0};
    double temperature_c{0.0};
    uint8_t error_flags{0};
  };

  struct GripperProtectionData
  {
    int over_current_ma{0};
    double over_temperature_c{0.0};
    double recovery_temperature_c{0.0};
  };

private:
  // Parameters for the Serial communications 
  std::string serial_port_path_ {"/dev/ttyUSB0"};
  std::string reader_port_path_ {"/dev/ttyUSB1"};
  std::string gripper_port_path_ {"/dev/ttyUSB2"};
  unsigned int baudrate_ {115200};
  unsigned int gripper_baudrate_ {115200};
  GripperBackend gripper_backend_{GripperBackend::LINEAR_ACTUATOR};
  uint8_t gripper_id_ = 1; 
  double dynamixel_protocol_version_{2.0};
  int32_t dynamixel_open_position_ticks_{2048};
  int32_t dynamixel_close_position_ticks_{2600};
  double dynamixel_goal_current_ma_{150.0};
  double dynamixel_open_current_ma_{150.0};
  double dynamixel_current_limit_ma_{500.0};
  double dynamixel_temperature_limit_c_{75.0};
  int dynamixel_bus_watchdog_ms_{200};
  bool dynamixel_configure_on_start_{true};
  bool dynamixel_apply_limits_on_start_{false};
  int32_t dynamixel_last_goal_position_ticks_{std::numeric_limits<int32_t>::min()};
  int dynamixel_last_goal_current_units_{std::numeric_limits<int>::min()};
  double pos_scale_ {1000.0}; // rad -> int16 scale
  std::array<double, 6> arm_joint_signs_ {{1.0, 1.0, -1.0, 1.0, 1.0, 1.0}};
  std::array<double, 6> arm_joint_offsets_ {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  double hw_slowdown_; // low-pass factor if no feedback 
  Stm32ControlMode stm32_control_mode_{Stm32ControlMode::POSITION_ONLY};
  std::array<double, 6> stm32_kp_{{50.0, 50.0, 50.0, 50.0, 50.0, 50.0}};
  std::array<double, 6> stm32_kd_{{2.0, 2.0, 2.0, 2.0, 2.0, 2.0}};
  std::array<double, 6> stm32_v_des_limits_rad_s_{{2.0, 2.0, 0.5, 2.0, 1.0, 2.0}};
  bool enable_arm_command_limiter_{true};
  std::array<double, 6> arm_command_velocity_limits_rad_s_{{2.0, 2.0, 0.5, 2.0, 1.0, 2.0}};
  std::array<double, 6> arm_command_acceleration_limits_rad_s2_{{1.0, 1.0, 0.2, 1.0, 0.5, 1.0}};
  std::array<double, 6> limited_arm_commands_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<double, 6> limited_arm_command_velocities_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  bool arm_command_limiter_ready_{false};
  std::mutex stm32_gain_mtx_;
  double stm32_heartbeat_duration_sec_{1.0};
  double stm32_trigger_duration_sec_{3.0};
  double stm32_feedback_wait_timeout_sec_{2.0};
  bool enable_stm32_zero_trigger_{false};
  bool stm32_recover_safe_drop_on_start_{true};
  double stm32_recover_duration_sec_{0.25};

  ArmCommandFrameFormat arm_command_frame_format_{ArmCommandFrameFormat::LEGACY_POSITION};
  bool enable_dynamics_feedforward_{false};
  DynamicsMode dynamics_mode_{DynamicsMode::GRAVITY};
  bool dynamics_use_commanded_position_{false};
  std::string dynamics_urdf_path_;
  std::string dynamics_feedforward_source_{"topic"};
  std::string dynamics_feedforward_topic_{"/arm_dynamics/torques_nm"};
  std::array<double, 6> dynamics_torque_scales_{{1.0, 1.0, 1.0, 1.0, 1.0, 1.0}};
  double dynamics_torque_low_pass_alpha_{0.2};
  bool dynamics_model_ready_{false};
  bool dynamics_warned_unavailable_{false};
  std::array<double, 6> dynamics_torque_limits_nm_{{0.5, 0.5, 0.3, 0.2, 0.15, 0.1}};
  std::array<double, 6> dynamics_torque_signs_{{1.0, 1.0, -1.0, 1.0, 1.0, 1.0}};
  std::array<double, 6> dynamics_last_tau_ros_nm_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<double, 6> dynamics_last_commands_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<double, 6> dynamics_last_command_velocities_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<double, 6> dynamics_topic_tau_ros_nm_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  rclcpp::Time dynamics_topic_last_time_{0, 0, RCL_ROS_TIME};
  double dynamics_topic_timeout_sec_{0.25};
  bool dynamics_command_history_ready_{false};
#ifdef MY_ARM_HARDWARE_HAS_PINOCCHIO
  pinocchio::Model dynamics_model_;
  std::unique_ptr<pinocchio::Data> dynamics_data_;
  std::array<int, 6> dynamics_joint_q_indices_{{-1, -1, -1, -1, -1, -1}};
  std::array<int, 6> dynamics_joint_v_indices_{{-1, -1, -1, -1, -1, -1}};
#endif

  // Objects for logging
  std::shared_ptr<rclcpp::Logger> logger_;
  rclcpp::Clock::SharedPtr clock_;

  // Store the command for the simulated robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_command_velocities_;
  std::vector<double> hw_states_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;
  
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
  dynamixel::PortHandler * dynamixel_port_{nullptr};
  dynamixel::PacketHandler * dynamixel_packet_{nullptr};

  // Used for reading data from the reader port
  std::vector<uint8_t> rx_buffer_;

  // Used for reading data from the gripper 
  std::vector<uint8_t> gripper_rx_;

  // Used for prevent initial jumps in position when no feedback is available
  bool initial_positions_received_ = false;
  double initial_read_timeout_sec_ = 2.0; 
  rclcpp::Time last_feedback_time_; // optional for diagnostics
  double feedback_stale_timeout_sec_ = 0.5;  // max allowed feedback age before writes stop
  std::array<double, 6> feedback_velocity_low_pass_alpha_{{1.0, 1.0, 1.0, 1.0, 1.0, 1.0}};
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

  int32_t grip_to_dynamixel_ticks(double ros) const;
  double dynamixel_ticks_to_grip(int32_t ticks) const;
  int dynamixel_current_ma_to_units(double current_ma) const;
  double dynamixel_current_units_to_ma(int current_units) const;

  std::vector<double> last_sent_commands_;
  void reset_joint_buffers(double value);
  void sync_commands_to_states();
  void parse_arm_command_limiter_parameters();
  void reset_arm_command_limiter_to_states();
  void apply_arm_command_limiter(const rclcpp::Duration & period);
  double arm_command_for_stm32(size_t joint_index) const;
  double arm_command_velocity_for_stm32(size_t joint_index) const;
  double filter_feedback_velocity(size_t joint_index, double raw_velocity) const;
  void publish_raw_feedback_velocities(const std::array<double, 6> & raw_velocities);
  size_t arm_joint_count() const;
  bool has_aux_joint() const;
  size_t aux_joint_index() const;
  void parse_gripper_parameters();
  void parse_dynamics_parameters();
  void parse_stm32_parameters();
  void initialize_dynamics_model();
  std::array<double, 6> compute_dynamics_torques(const rclcpp::Duration & period);
  void publish_dynamics_torques(const std::array<double, 6> & tau_ros_nm);
  void open_stm32_trace();
  void close_stm32_trace();
  void trace_stm32_sample(
    const std::string & event,
    uint8_t mode,
    uint8_t sys_state,
    const std::array<double, 6> & tau_ros_nm,
    const std::array<double, 6> & tau_hw_nm);
  std::vector<uint8_t> build_position_command_frame();
  std::vector<uint8_t> build_position_torque_command_frame(const std::array<double, 6> & tau_hw_nm);
  std::vector<uint8_t> build_stm32_command_frame(
    uint8_t mode,
    const std::array<double, 6> & tau_hw_nm,
    bool zero_payload = false);
  std::vector<uint8_t> build_stm32_trigger_frame();
  bool send_stm32_frame(const std::vector<uint8_t> & frame);
  bool pump_stm32_boot_phase(
    const std::vector<uint8_t> & frame,
    double duration_sec,
    const std::string & phase_name);
  bool wait_for_stm32_feedback_and_sync(double timeout_sec);
  void setup_homing_interface();
  void teardown_homing_interface();
  void publish_homing_state(RosMasterHomingState state, const std::string & message);
  void publish_stm32_sys_state(uint8_t sys_state);
  void publish_stm32_control_mode();
  void publish_stm32_mit_gains();
  void apply_stm32_mit_gains_command(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void handle_system_event(uint8_t event_code);
  bool send_system_command(uint8_t command_code, std::string & failure_reason);
  bool gripper_query_status(GripperStatusData & status, std::string & message);
  bool gripper_query_protection(GripperProtectionData & protection, std::string & message);
  bool gripper_clear_fault(std::string & message);
  bool gripper_clear_fault_and_open(std::string & message);
  bool gripper_save_parameters(std::string & message);
  bool gripper_set_protection(
    int over_current_ma,
    double over_temperature_c,
    double recovery_temperature_c,
    bool save_to_flash,
    GripperProtectionData & applied,
    std::string & message);
  bool gripper_send_frame_locked(const std::vector<uint8_t> & frame, std::string & message);
  bool gripper_read_frame_locked(
    std::vector<uint8_t> & frame,
    std::chrono::milliseconds timeout,
    std::string & message);
  bool gripper_send_single_control_locked(uint8_t command, std::string & message);
  bool gripper_read_u16_register_locked(
    uint8_t address,
    uint16_t & value,
    std::string & message);
  bool gripper_write_u16_register_locked(
    uint8_t address,
    uint16_t value,
    std::string & message);
  void update_gripper_status_from_frame(const std::vector<uint8_t> & frame);
  bool dynamixel_open_locked(std::string & message);
  void dynamixel_close_locked();
  bool dynamixel_initialize_locked(std::string & message);
  bool dynamixel_query_status_locked(GripperStatusData & status, std::string & message);
  bool dynamixel_command_position_locked(
    int32_t goal_ticks,
    double current_ma,
    bool force,
    std::string & message);
  bool dynamixel_recover_locked(std::string & message);
  bool dynamixel_read_protection_locked(GripperProtectionData & protection, std::string & message);
  bool dynamixel_write_protection_locked(
    int over_current_ma,
    double over_temperature_c,
    std::string & message);
  bool dynamixel_write1_locked(
    uint16_t address,
    uint8_t value,
    const std::string & label,
    std::string & message);
  bool dynamixel_write2_locked(
    uint16_t address,
    uint16_t value,
    const std::string & label,
    std::string & message);
  bool dynamixel_write4_locked(
    uint16_t address,
    uint32_t value,
    const std::string & label,
    std::string & message);
  bool dynamixel_read1_locked(
    uint16_t address,
    uint8_t & value,
    const std::string & label,
    std::string & message);
  bool dynamixel_read2_locked(
    uint16_t address,
    uint16_t & value,
    const std::string & label,
    std::string & message);
  bool dynamixel_read4_locked(
    uint16_t address,
    uint32_t & value,
    const std::string & label,
    std::string & message);
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
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr stm32_sys_state_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr stm32_control_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr stm32_mit_gains_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr raw_feedback_velocity_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr dynamics_tau_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr dynamics_tau_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr stm32_mit_gains_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr homing_init_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr homing_drop_pose_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr homing_init_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr homing_confirm_srv_;
  rclcpp::Service<my_arm_hardware::srv::GripperStatus>::SharedPtr gripper_status_srv_;
  rclcpp::Service<my_arm_hardware::srv::GetGripperProtection>::SharedPtr gripper_protection_get_srv_;
  rclcpp::Service<my_arm_hardware::srv::SetGripperProtection>::SharedPtr gripper_protection_set_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr gripper_clear_fault_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr gripper_clear_fault_open_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr gripper_save_parameters_srv_;
  std::mutex homing_state_mtx_;
  std::mutex gripper_status_mtx_;
  GripperStatusData last_gripper_status_;
  RosMasterHomingState homing_state_{RosMasterHomingState::WAITING_INIT_COMMAND};
  uint8_t last_stm32_sys_state_{std::numeric_limits<uint8_t>::max()};
  uint8_t last_stm32_write_mode_{std::numeric_limits<uint8_t>::max()};
  std::array<double, 6> last_raw_feedback_velocities_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<double, 6> last_tau_ros_nm_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<double, 6> last_tau_hw_nm_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  bool stm32_trace_enabled_{false};
  std::string stm32_trace_directory_{"/tmp/arm_stm32_traces"};
  std::string stm32_trace_run_label_;
  std::string stm32_trace_file_;
  bool stm32_trace_append_{false};
  size_t stm32_trace_decimation_{1};
  size_t stm32_trace_counter_{0};
  std::ofstream stm32_trace_stream_;
  std::mutex stm32_trace_mtx_;
  std::atomic_bool stm32_homing_trigger_active_{false};
  bool sync_commands_on_next_feedback_{false};
  std::mutex command_hold_mtx_;
  std::vector<double> post_homing_hold_commands_;
  bool post_homing_command_hold_active_{false};
};

}  // namespace my_arm_hardware
