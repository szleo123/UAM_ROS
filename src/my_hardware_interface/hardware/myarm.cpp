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

#include "my_arm_hardware/myarm.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#ifdef MY_ARM_HARDWARE_HAS_PINOCCHIO
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>
#endif
#include "pluginlib/class_list_macros.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace
{
constexpr size_t kArmFeedbackJointCount = 6;
constexpr uint8_t kArmFeedbackHeader = 0xAA;
constexpr uint8_t kArmCommandHeader = 0xFF;
constexpr uint8_t kArmPositionTorqueCommandHeader = 0xFE;
constexpr uint8_t kSystemCommandHeader = 0xEE;
constexpr size_t kSystemFrameLength = 14;
constexpr uint8_t kEventDamiaoReady = 0x01;
constexpr uint8_t kEventAllReady = 0x02;
constexpr uint8_t kCmdReachedDropPose = 0x01;
constexpr uint8_t kCmdStartDamiaoInit = 0x10;
constexpr size_t kArmFrameLength = 14;
constexpr size_t kArmPositionTorqueFrameLength = 38;
constexpr double kMinPeriodSec = 1e-6;
constexpr double kGripperUnitsMin = 60.0;
constexpr double kGripperUnitsMax = 1390.0;
constexpr const char * kHomingStatusTopic = "/arm_homing/status_text";
constexpr const char * kHomingStateTopic = "/arm_homing/state";
constexpr const char * kHomingInitService = "/arm_homing/start_initialization";
constexpr const char * kHomingInitTopic = "/arm_homing/start_initialization";
constexpr const char * kHomingConfirmService = "/arm_homing/confirm_drop_pose";
constexpr const char * kHomingConfirmTopic = "/arm_homing/reached_drop_pose";

double safe_period_seconds(const rclcpp::Duration & period)
{
  return std::max(period.seconds(), kMinPeriodSec);
}

template <typename VectorT>
void reset_vector(VectorT & values, double value)
{
  std::fill(values.begin(), values.end(), value);
}

uint8_t frame_checksum(const std::vector<uint8_t> & frame)
{
  uint32_t sum = 0;
  for (size_t i = 0; i + 1 < frame.size(); ++i)
    sum += frame[i];
  return static_cast<uint8_t>(sum & 0xFF);
}

}  // namespace

static LibSerial::BaudRate baud_from_uint(unsigned int b)
{
  using BR = LibSerial::BaudRate;
  switch (b) {
    case 50: return BR::BAUD_50;
    case 75: return BR::BAUD_75;
    case 110: return BR::BAUD_110;
    case 134: return BR::BAUD_134;
    case 150: return BR::BAUD_150;
    case 200: return BR::BAUD_200;
    case 300: return BR::BAUD_300;
    case 600: return BR::BAUD_600;
    case 1200: return BR::BAUD_1200;
    case 1800: return BR::BAUD_1800;
    case 2400: return BR::BAUD_2400;
    case 4800: return BR::BAUD_4800;
    case 9600: return BR::BAUD_9600;
    case 19200: return BR::BAUD_19200;
    case 38400: return BR::BAUD_38400;
    case 57600: return BR::BAUD_57600;
    case 115200: return BR::BAUD_115200;
    case 230400: return BR::BAUD_230400;
    default: throw std::invalid_argument("Unsupported baudrate: " + std::to_string(b));
  }
}

namespace
{
void open_serial_port(
  LibSerial::SerialPort & port,
  const std::string & path,
  unsigned int baudrate)
{
  if (port.IsOpen())
    port.Close();

  port.Open(path);
  port.SetBaudRate(baud_from_uint(baudrate));
  port.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
  port.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
  port.SetParity(LibSerial::Parity::PARITY_NONE);
  port.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
}

void close_serial_port(LibSerial::SerialPort & port)
{
  if (!port.IsOpen())
    return;
  port.FlushIOBuffers();
  port.Close();
}
}  // namespace

static inline uint8_t gripper_checksum(const std::vector<uint8_t>& f, size_t start=2) {
  uint32_t s = 0; for (size_t i = start; i < f.size(); ++i) s += f[i];
  return static_cast<uint8_t>(s & 0xFF);
}

// Build "positioning with feedback" frame: 55 AA 04 [ID] 21 37 [lo] [hi] [chk]
static inline std::vector<uint8_t> gripper_pack_target(uint8_t id, uint16_t target_units) {
  std::vector<uint8_t> f{0x55,0xAA,0x04,id,0x21,0x37,
                         static_cast<uint8_t>(target_units & 0xFF),
                         static_cast<uint8_t>((target_units >> 8) & 0xFF)};
  f.push_back(gripper_checksum(f));
  return f;
}


/* RX parser: look for AA 55 ... reply that contains current position.
   We check header, length, checksum, and read current position from payload.
   Adjust the offsets if your device's state frame differs. */
static inline bool gripper_try_parse_state(std::vector<uint8_t>& buf, int16_t& current_units) {
  // seek header
  size_t i = 0;
  while (buf.size() - i >= 9) {
    if (buf[i] == 0xAA && buf[i+1] == 0x55)
    {
      if (buf.size() - i < 4) break; 
      uint8_t len = buf[i+2];                       // payload count after [len]
      size_t frame_len = 2 + 1 + (len + 1) + 1;     // hdr(2)+len(1)+[ID..payload..](len+1)+chk(1)

      // checksum over bytes starting at [len]
      uint32_t s = 0; for (size_t k = i+2; k < i+frame_len-1; ++k) s += buf[k];
      uint8_t chk = buf[i + frame_len - 1];
      if (chk != static_cast<uint8_t>(s & 0xFF)) { ++i; continue; }
      if (buf[i+4] != 0x21 || buf[i+5] != 0x37) { ++i; continue; } // not a state frame

      current_units = static_cast<int16_t>(buf[i+9] | buf[i+10] << 8);
      buf.erase(buf.begin()+static_cast<long>(i), buf.begin()+static_cast<long>(i+frame_len));
      return true;
    }
    

    // Not a state frame we need; drop one byte and resync
    ++i;
  }
  // drop consumed prefix
  if (i > 0) buf.erase(buf.begin(), buf.begin()+static_cast<long>(i));
  return false;
}

static inline std::vector<uint8_t> pack_system_command(uint8_t command_code)
{
  std::vector<uint8_t> frame(kSystemFrameLength, 0);
  frame[0] = kSystemCommandHeader;
  frame[1] = command_code;
  frame.back() = frame_checksum(frame);
  return frame;
}

static inline std::string trim_copy(const std::string &input)
{
  const auto start = input.find_first_not_of(" \t\n\r");
  if (start == std::string::npos)
    return "";
  const auto end = input.find_last_not_of(" \t\n\r");
  return input.substr(start, end - start + 1);
}

static inline bool string_to_bool(const std::string &value)
{
  std::string lowered;
  lowered.reserve(value.size());
  for (char c : value)
    lowered.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
  return lowered == "1" || lowered == "true" || lowered == "on" || lowered == "yes";
}

static inline std::vector<std::string> split_list(const std::string &csv)
{
  std::string sanitized = trim_copy(csv);
  if (!sanitized.empty() && sanitized.front() == '[')
    sanitized.erase(sanitized.begin());
  if (!sanitized.empty() && sanitized.back() == ']')
    sanitized.pop_back();
  std::vector<std::string> entries;
  std::stringstream ss(sanitized);
  std::string token;
  while (std::getline(ss, token, ','))
  {
    auto trimmed = trim_copy(token);
    if (!trimmed.empty())
      entries.push_back(trimmed);
  }
  return entries;
}

static inline void append_i16_le(std::vector<uint8_t> & frame, int16_t value)
{
  frame.push_back(static_cast<uint8_t>(value & 0xFF));
  frame.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
}

static inline void append_f32_le(std::vector<uint8_t> & frame, float value)
{
  uint32_t bits = 0;
  static_assert(sizeof(bits) == sizeof(value), "float32 packing assumes 32-bit float");
  std::memcpy(&bits, &value, sizeof(value));
  frame.push_back(static_cast<uint8_t>(bits & 0xFF));
  frame.push_back(static_cast<uint8_t>((bits >> 8) & 0xFF));
  frame.push_back(static_cast<uint8_t>((bits >> 16) & 0xFF));
  frame.push_back(static_cast<uint8_t>((bits >> 24) & 0xFF));
}

namespace my_arm_hardware
{
bool MyArmHardware::wait_for_initial_feedback(std::array<double,6>& q, double timeout_sec)
{
  rclcpp::Time t0 = get_clock()->now();
  std::vector<uint8_t> buf_local; buf_local.reserve(128);

  while ((get_clock()->now() - t0).seconds() < timeout_sec)
  {
    try {
      std::scoped_lock lk(reader_mtx_);
      while (reader_.IsDataAvailable())
      {
        char c = 0;
        reader_.ReadByte(c, 2);  // small blocking (2ms) to coalesce bytes
        buf_local.push_back(static_cast<uint8_t>(c));
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "initial feedback read error: %s", e.what());
      return false;
    }

    // scan for frames
    size_t i = 0;
    while (buf_local.size() - i >= kSystemFrameLength)
    {
      const uint8_t header = buf_local[i];
      if (header != kArmFeedbackHeader && header != kSystemCommandHeader) {
        ++i;
        continue;
      }
      if (buf_local.size() - i < kSystemFrameLength) break;

      uint32_t sum = 0;
      for (size_t k = 0; k < kSystemFrameLength - 1; ++k) sum += buf_local[i + k];
      uint8_t cs = static_cast<uint8_t>(sum & 0xFF);
      uint8_t rxcs = buf_local[i + kSystemFrameLength - 1];

      if (cs != rxcs) {
        ++i;
        continue;
      }

      if (header == kSystemCommandHeader)
      {
        handle_system_event(buf_local[i + 1]);
        i += kSystemFrameLength;
        continue;
      }

      // decode 6 int16 LE -> double
      for (size_t j = 0; j < kArmFeedbackJointCount; ++j)
      {
        uint8_t lo = buf_local[i + 1 + 2*j];
        uint8_t hi = buf_local[i + 1 + 2*j + 1];
        int16_t raw = static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) | lo);
        q[j] = arm_joint_signs_[j] * static_cast<double>(raw) / pos_scale_ +
               arm_joint_offsets_[j];
      }
      return true;
    }

    // brief sleep to avoid busy spin
    rclcpp::sleep_for(std::chrono::milliseconds(2));
  }
  return false; // timeout
}

hardware_interface::CallbackReturn MyArmHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger("my_arm_system"));
  clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock());

  // Parse params from <ros2_control> hardware <param>
  if (info_.hardware_parameters.count("serial_port"))
    serial_port_path_ = info_.hardware_parameters.at("serial_port");

  if (info_.hardware_parameters.count("reader_port"))
    reader_port_path_ = info_.hardware_parameters.at("reader_port");

  if (info_.hardware_parameters.count("gripper_port"))
    gripper_port_path_ = info_.hardware_parameters.at("gripper_port");

  if (info_.hardware_parameters.count("baudrate"))
    baudrate_ = static_cast<unsigned int>(std::stoi(info_.hardware_parameters.at("baudrate")));

  if (info_.hardware_parameters.count("gripper_baudrate"))
    gripper_baudrate_ = static_cast<unsigned int>(std::stoi(info_.hardware_parameters.at("gripper_baudrate")));

  if (info_.hardware_parameters.count("position_scale"))
    pos_scale_ = std::stod(info_.hardware_parameters.at("position_scale"));
  if (info_.hardware_parameters.count("arm_joint_signs"))
  {
    const auto signs = split_list(info_.hardware_parameters.at("arm_joint_signs"));
    for (size_t i = 0; i < signs.size() && i < arm_joint_signs_.size(); ++i)
    {
      const double sign = std::stod(signs[i]);
      if (std::abs(sign) <= std::numeric_limits<double>::epsilon())
      {
        RCLCPP_WARN(get_logger(), "arm_joint_signs[%zu] is zero; keeping %.1f.", i, arm_joint_signs_[i]);
        continue;
      }
      arm_joint_signs_[i] = sign < 0.0 ? -1.0 : 1.0;
    }
  }
  if (info_.hardware_parameters.count("arm_joint_offsets"))
  {
    const auto offsets = split_list(info_.hardware_parameters.at("arm_joint_offsets"));
    for (size_t i = 0; i < offsets.size() && i < arm_joint_offsets_.size(); ++i)
      arm_joint_offsets_[i] = std::stod(offsets[i]);
  }

  if (info_.hardware_parameters.count("hw_slowdown"))
    hw_slowdown_ = std::stod(info_.hardware_parameters.at("hw_slowdown"));
  else
    hw_slowdown_ = 1.0;

  if (info_.hardware_parameters.count("initial_read_timeout_sec"))
    initial_read_timeout_sec_ = std::stod(info_.hardware_parameters.at("initial_read_timeout_sec"));
  if (info_.hardware_parameters.count("feedback_stale_timeout_sec"))
    feedback_stale_timeout_sec_ = std::stod(info_.hardware_parameters.at("feedback_stale_timeout_sec"));
  if (info_.hardware_parameters.count("first_power_on"))
    first_power_on_ = string_to_bool(info_.hardware_parameters.at("first_power_on"));
  if (info_.hardware_parameters.count("aux_joint_min"))
    aux_joint_min_ = std::stod(info_.hardware_parameters.at("aux_joint_min"));
  if (info_.hardware_parameters.count("aux_joint_max"))
    aux_joint_max_ = std::stod(info_.hardware_parameters.at("aux_joint_max"));

  parse_dynamics_parameters();

  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  last_sent_commands_.assign(hw_states_.size(), std::numeric_limits<double>::quiet_NaN());

  // Validate joint interfaces (position only)
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1 || 
        joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' must expose exactly one '%s' command interfaces.",
        joint.name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces.empty() || 
        joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
        joint.state_interfaces.size() > 2)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' must expose exactly one '%s' state interfaces.",
        joint.name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(get_logger(),
              "Initialized MyArmSystem with %i joints, writer_port=%s, reader_port=%s, baud=%u, scale=%.1f, slowdown=%.1f, first_power_on=%s, command_frame=%s, dynamics_feedforward=%s, dynamics_mode=%s, arm_joint_offsets=[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f]",
              static_cast<int>(info_.joints.size()), serial_port_path_.c_str(),
              reader_port_path_.c_str(), baudrate_, pos_scale_, hw_slowdown_,
              first_power_on_ ? "true" : "false",
              arm_command_frame_format_ == ArmCommandFrameFormat::POSITION_TORQUE ? "position_torque" : "legacy_position",
              enable_dynamics_feedforward_ ? "true" : "false",
              dynamics_mode_ == DynamicsMode::FULL ? "full" :
                (dynamics_mode_ == DynamicsMode::CORIOLIS ? "coriolis" : "gravity"),
              arm_joint_offsets_[0], arm_joint_offsets_[1], arm_joint_offsets_[2],
              arm_joint_offsets_[3], arm_joint_offsets_[4], arm_joint_offsets_[5]);
  setup_homing_interface();
  if (enable_dynamics_feedforward_)
    initialize_dynamics_model();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MyArmHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring ...please wait...");
  try 
  {
    std::scoped_lock lk(serial_mtx_);
    open_serial_port(serial_, serial_port_path_, baudrate_);
    serial_ok_ = true;
    RCLCPP_INFO(get_logger(), "Serial port %s opened at %u baud", 
                serial_port_path_.c_str(), baudrate_);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_logger(), "Error configuring serial port %s: %s", 
                 serial_port_path_.c_str(), e.what());
    serial_ok_ = false;
  }

  try
  {
    std::scoped_lock rk(reader_mtx_);
    open_serial_port(reader_, reader_port_path_, baudrate_);
    reader_ok_ = true;
    RCLCPP_INFO(get_logger(), "Reader port %s opened at %u baud", 
                reader_port_path_.c_str(), baudrate_);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_logger(), "Error configuring reader port %s: %s", 
                 reader_port_path_.c_str(), e.what());
    reader_ok_ = false;
  }

  try 
  {
    std::scoped_lock gk(gripper_mtx_);
    open_serial_port(gripper_, gripper_port_path_, gripper_baudrate_);
    gripper_ok_ = true;
    gripper_.FlushIOBuffers();
    gripper_rx_.clear();
    RCLCPP_INFO(get_logger(), "Gripper port %s opened at %u baud", 
                gripper_port_path_.c_str(), gripper_baudrate_);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_logger(), "Error configuring gripper port %s: %s", 
                 gripper_port_path_.c_str(), e.what());
    gripper_ok_ = false;
  }

  reset_joint_buffers(0.0);
  rx_buffer_.clear();
  sync_commands_on_next_feedback_ = false;
  if (first_power_on_)
  {
    publish_homing_state(
      RosMasterHomingState::WAITING_INIT_COMMAND,
      "Waiting for operator initialization command. Click Initialize Damiao or call "
      "/arm_homing/start_initialization before enabling the Damiao motors.");
  }
  else
  {
    publish_homing_state(
      RosMasterHomingState::ALL_READY,
      "first_power_on is false: skipping ROS-master homing handshake. Motion writes "
      "will still wait for fresh hardware feedback and sync commands to measured positions.");
  }

  RCLCPP_INFO(get_logger(), "Successfully configured!");
  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn MyArmHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Activating... waiting for initial feedback up to %.2fs",
              initial_read_timeout_sec_);

  std::array<double,6> q{};
  if (reader_ok_ && wait_for_initial_feedback(q, initial_read_timeout_sec_))
  {
    // copy to states/commands (only up to the number of joints you expose)
    for (size_t i = 0; i < hw_states_.size() && i < q.size(); ++i) {
      hw_states_[i]   = q[i];
      hw_commands_[i] = q[i];
    }
    initial_positions_received_ = true;
    last_feedback_time_ = get_clock()->now();
    RCLCPP_WARN(get_logger(), "Initial positions synced from hardware. Writes now enabled.");
  }
  else
  {
    // No feedback—**do not** send commands yet. Controller can start, but write() is gated.
    initial_positions_received_ = false;
    sync_commands_to_states();
    RCLCPP_ERROR(get_logger(),
      "Initial feedback NOT received within %.2fs. write() will be skipped until feedback arrives.",
      initial_read_timeout_sec_);
  }

  last_sent_commands_ = hw_commands_;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MyArmHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");

  // Freeze commands at current positions
  sync_commands_to_states();
  RCLCPP_INFO(get_logger(), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
MyArmHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
MyArmHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}



hardware_interface::return_type MyArmHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{

  if (!reader_ok_ && !gripper_ok_)
  {
    const double dt = safe_period_seconds(period);
    for (size_t i = 0; i < hw_states_.size(); ++i)
    {
      double prev = hw_states_[i];
      hw_states_[i] = hw_commands_[i];  // snap to command for perfect sim
      hw_velocities_[i] = (hw_states_[i] - prev) / dt; // calculate velocity
    }

    RCLCPP_WARN_THROTTLE(get_logger(), *clock_, 2000, "Readers not OK; skipping read().");
    return hardware_interface::return_type::OK;
  } 
  
  if (reader_ok_)
  {
    // 1 pull in all available data from the reader port without blocking
    try 
    {
      std::scoped_lock rk(reader_mtx_);
      // ReadByte(c, timeout_ms) - use 0ms to be non-blocking 
      char c = 0; 
      while (reader_.IsDataAvailable())
      {
        reader_.ReadByte(c, 0); // retrn immediately
        rx_buffer_.push_back(static_cast<uint8_t>(c));
      }
    } catch (const std::exception & e)
    {
      reader_ok_ = false;
      RCLCPP_ERROR_THROTTLE(get_logger(), *clock_, 2000, "Reader read failed: %s", e.what());
      return hardware_interface::return_type::OK;
    }
    // 2 Parse frames: 0xAA + 12 data bytes + checksum
    size_t i = 0; 
    bool frame_received = false;
    const double dt = safe_period_seconds(period);
    while (rx_buffer_.size() - i >= kSystemFrameLength) {
      const uint8_t header = rx_buffer_[i];
      if (header != kArmFeedbackHeader && header != kSystemCommandHeader) {
        i++;
        continue;
      }

      if (rx_buffer_.size() - i < kSystemFrameLength) {
        break; // wait for more data
      }

      uint32_t sum = 0; 
      for (size_t k = 0; k < kSystemFrameLength - 1; k++) {
        sum += rx_buffer_[i + k];
      }
      uint8_t checksum = static_cast<uint8_t>(sum & 0xFF);
      uint8_t rx_checksum = rx_buffer_[i + kSystemFrameLength - 1];

      if (checksum != rx_checksum) {
        i++;
        continue;
      }

      if (header == kSystemCommandHeader)
      {
        handle_system_event(rx_buffer_[i + 1]);
        i += kSystemFrameLength;
        continue;
      }

      // Valid frame: decode 6 little-endian int16 angles 
      const size_t joints_to_decode = std::min(arm_joint_count(), kArmFeedbackJointCount);
      for (size_t j = 0; j < joints_to_decode; j++) {
        uint8_t lo = rx_buffer_[i + 1 + 2*j];
        uint8_t hi = rx_buffer_[i + 1 + 2*j + 1];
        int16_t raw = static_cast<int16_t>(static_cast<uint16_t>(hi) << 8 | lo);
        double angle = arm_joint_signs_[j] * static_cast<double>(raw) / pos_scale_ +
                       arm_joint_offsets_[j];
        double prev = hw_states_[j];
        hw_states_[j] = angle;
        hw_velocities_[j] = (hw_states_[j] - prev) / dt;
      }

      i += kArmFrameLength; // advance past this frame

      // if this is the first valid feedback, mark it
      last_feedback_time_ = get_clock()->now();
      frame_received = true;
      if (!initial_positions_received_) {
        sync_commands_to_states();
        initial_positions_received_ = true;
        RCLCPP_WARN(get_logger(),
          "Initial positions synced from hardware. Writes now enabled.");
      }
      if (sync_commands_on_next_feedback_) {
        sync_commands_to_states();
        last_sent_commands_ = hw_commands_;
        sync_commands_on_next_feedback_ = false;
        RCLCPP_WARN(get_logger(),
          "Homing complete feedback received. Commands resynced to measured positions.");
      }
      if (arm_joint_count() >= kArmFeedbackJointCount) {
        RCLCPP_INFO_THROTTLE(get_logger(), *clock_, 1000, 
          "Read positions: %.3f %.3f %.3f %.3f %.3f %.3f",
          hw_states_[0], hw_states_[1], hw_states_[2],
          hw_states_[3], hw_states_[4], hw_states_[5]);
      }
    }

    // Erase all processed bytes
    if (i > 0) {
      rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + static_cast<long>(i));
    }

    if (!frame_received && initial_positions_received_ &&
        feedback_stale_timeout_sec_ > 1e-6 &&
        last_feedback_time_.nanoseconds() > 0)
    {
      const double age = (get_clock()->now() - last_feedback_time_).seconds();
      if (age > feedback_stale_timeout_sec_)
      {
        initial_positions_received_ = false;
        RCLCPP_ERROR_THROTTLE(get_logger(), *clock_, 2000,
          "Feedback stale: %.0f ms since last update (limit %.0f ms). Writes disabled until feedback resumes.",
          age * 1000.0, feedback_stale_timeout_sec_ * 1000.0);
      }
    }

  }

  if (gripper_ok_) {
    try {
      std::scoped_lock gk(gripper_mtx_);
      // pull in all available bytes (non-blocking)
      char c = 0;
      while (gripper_.IsDataAvailable()) {
        gripper_.ReadByte(c, 0);
        gripper_rx_.push_back(static_cast<uint8_t>(c));
      }
      // parse all complete state frames
      int16_t cur_units = 0;
      if (gripper_try_parse_state(gripper_rx_, cur_units)) {
        double ros_pos = units_to_grip(cur_units);
        if (has_aux_joint()) {
          const size_t aux_idx = aux_joint_index();
          double prev = hw_states_[aux_idx];
          hw_states_[aux_idx] = ros_pos;
          hw_velocities_[aux_idx] = (hw_states_[aux_idx] - prev) / safe_period_seconds(period);
        }
        // (Optional)
        RCLCPP_INFO_THROTTLE(get_logger(), *clock_, 2000,
          "Gripper read: units %d -> ROS %.3f", (int)cur_units, ros_pos);
      }

    } catch (const std::exception& e) {
      gripper_ok_ = false;
      RCLCPP_ERROR_THROTTLE(get_logger(), *clock_, 2000,
        "Gripper read failed: %s", e.what());
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MyArmHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!serial_ok_ && !gripper_ok_)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *clock_, 2000, "Serial not OK; skipping write().");
    return hardware_interface::return_type::OK;
  }

  enforce_post_homing_command_hold();

  if (!can_write_motion_commands())
  {
    RosMasterHomingState homing_state;
    {
      std::lock_guard<std::mutex> lock(homing_state_mtx_);
      homing_state = homing_state_;
    }
    if (homing_state != RosMasterHomingState::WAITING_ALL_READY)
      sync_commands_to_states();
    RCLCPP_WARN_THROTTLE(get_logger(), *clock_, 2000,
      "Skipping write(): ROS-master homing has not released motion commands yet.");
    return hardware_interface::return_type::OK;
  }

  // ---- Gripper TX (full-duplex same port) ----
  if (gripper_ok_ && has_aux_joint()) {
    const size_t aux_idx = aux_joint_index();
    const double ros_cmd = hw_commands_[aux_idx];              // ROS units (e.g., radians)
    uint16_t tgt = grip_to_units(ros_cmd);
    try {
      std::scoped_lock gk(gripper_mtx_);
      auto f = gripper_pack_target(gripper_id_, tgt);
      gripper_.Write(f);
      gripper_.DrainWriteBuffer();
      // (Optional throttle)
      RCLCPP_INFO_THROTTLE(get_logger(), *clock_, 2000,
        "Gripper write: ROS %.3f -> units %u", ros_cmd, (unsigned)tgt);
    } catch (const std::exception& e) {
      gripper_ok_ = false;
      RCLCPP_ERROR_THROTTLE(get_logger(), *clock_, 2000,
        "Gripper write failed: %s", e.what());
    }
    
  }

  if (serial_ok_)
  {
    if (!initial_positions_received_)
    {
      double age_ms = 0.0;
      if (last_feedback_time_.nanoseconds() > 0) {
        age_ms = std::max(0.0,
          (get_clock()->now() - last_feedback_time_).seconds() * 1000.0);
      }
      RCLCPP_WARN_THROTTLE(get_logger(), *clock_, 2000,
        "Skipping write(): waiting for fresh feedback (last update %.0f ms ago).",
        age_ms);
      return hardware_interface::return_type::OK;
    }
    if (!reader_ok_)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *clock_, 2000,
        "Reader not OK; skipping write().");
      return hardware_interface::return_type::OK;
    }
    std::array<double, 6> tau_ros_nm{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    std::array<double, 6> tau_hw_nm{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    std::vector<uint8_t> frame;
    if (arm_command_frame_format_ == ArmCommandFrameFormat::POSITION_TORQUE)
    {
      tau_ros_nm = compute_dynamics_torques(period);
      for (size_t i = 0; i < tau_hw_nm.size(); ++i)
        tau_hw_nm[i] = dynamics_torque_signs_[i] * tau_ros_nm[i];
      frame = build_position_torque_command_frame(tau_hw_nm);
      publish_dynamics_torques(tau_ros_nm);
    }
    else
    {
      frame = build_position_command_frame();
    }

    try
    {
      std::scoped_lock lk(serial_mtx_);
      serial_.Write(frame);
      // RCLCPP_INFO_THROTTLE(get_logger(), *clock_, 1000, "write() OK");
      if (arm_joint_count() >= kArmFeedbackJointCount) {
        RCLCPP_INFO_THROTTLE(get_logger(), *clock_, 1000, 
            "Write positions: %.3f %.3f %.3f %.3f %.3f %.3f",
            hw_commands_[0], hw_commands_[1], hw_commands_[2],
            hw_commands_[3], hw_commands_[4], hw_commands_[5]);
        if (arm_command_frame_format_ == ArmCommandFrameFormat::POSITION_TORQUE) {
          RCLCPP_INFO_THROTTLE(get_logger(), *clock_, 1000,
              "Write tau_ff ROS Nm: %.3f %.3f %.3f %.3f %.3f %.3f",
              tau_ros_nm[0], tau_ros_nm[1], tau_ros_nm[2],
              tau_ros_nm[3], tau_ros_nm[4], tau_ros_nm[5]);
        }
      }
    }
    catch (const std::exception & e)
    {
      serial_ok_ = false;
      RCLCPP_ERROR_THROTTLE(get_logger(), *clock_, 2000, "Serial write failed: %s", e.what());
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn
MyArmHardware::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up (de-configure) ...");

  teardown_homing_interface();

  // Close writer port
  try {
    std::scoped_lock lk(serial_mtx_);
    close_serial_port(serial_);
    serial_ok_ = false;
  } catch (const std::exception &e) {
    RCLCPP_WARN(get_logger(), "Error while closing serial writer: %s", e.what());
  }

  // Close reader port (if you use a dedicated one)
  try {
    std::scoped_lock rk(reader_mtx_);
    close_serial_port(reader_);
    reader_ok_ = false;
  } catch (const std::exception &e) {
    RCLCPP_WARN(get_logger(), "Error while closing serial reader: %s", e.what());
  }

  try {
    std::scoped_lock gk(gripper_mtx_);
    close_serial_port(gripper_);
    gripper_ok_ = false;
    gripper_rx_.clear();
  } catch (const std::exception &e) {
    RCLCPP_WARN(get_logger(), "Error while closing gripper serial port: %s", e.what());
  }

  // Clear any accumulated RX parsing buffer
  {
    std::scoped_lock rk(reader_mtx_);
    rx_buffer_.clear();
  }

  RCLCPP_INFO(get_logger(), "Cleanup complete.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
MyArmHardware::on_shutdown(const rclcpp_lifecycle::State &)
{
  // Reuse on_cleanup to close ports
  (void)on_cleanup(rclcpp_lifecycle::State());
  return hardware_interface::CallbackReturn::SUCCESS;
}

void MyArmHardware::reset_joint_buffers(double value)
{
  reset_vector(hw_states_, value);
  reset_vector(hw_commands_, value);
  reset_vector(hw_velocities_, value);
}

void MyArmHardware::sync_commands_to_states()
{
  if (hw_commands_.size() != hw_states_.size())
    hw_commands_.resize(hw_states_.size(), 0.0);
  std::copy(hw_states_.begin(), hw_states_.end(), hw_commands_.begin());
}

size_t MyArmHardware::arm_joint_count() const
{
  return std::min(info_.joints.size(), kArmFeedbackJointCount);
}

bool MyArmHardware::has_aux_joint() const
{
  return info_.joints.size() > kArmFeedbackJointCount;
}

size_t MyArmHardware::aux_joint_index() const
{
  return info_.joints.empty() ? 0 : info_.joints.size() - 1;
}

void MyArmHardware::parse_dynamics_parameters()
{
  if (info_.hardware_parameters.count("arm_command_frame_format"))
  {
    const std::string format = trim_copy(info_.hardware_parameters.at("arm_command_frame_format"));
    if (format == "position_torque" || format == "extended_position_torque")
      arm_command_frame_format_ = ArmCommandFrameFormat::POSITION_TORQUE;
    else if (format == "legacy_position" || format == "position")
      arm_command_frame_format_ = ArmCommandFrameFormat::LEGACY_POSITION;
    else
      RCLCPP_WARN(get_logger(), "Unknown arm_command_frame_format '%s'; using legacy_position.", format.c_str());
  }

  if (info_.hardware_parameters.count("enable_dynamics_feedforward"))
    enable_dynamics_feedforward_ = string_to_bool(info_.hardware_parameters.at("enable_dynamics_feedforward"));

  if (info_.hardware_parameters.count("dynamics_mode"))
  {
    const std::string mode = trim_copy(info_.hardware_parameters.at("dynamics_mode"));
    if (mode == "full")
      dynamics_mode_ = DynamicsMode::FULL;
    else if (mode == "coriolis")
      dynamics_mode_ = DynamicsMode::CORIOLIS;
    else if (mode == "gravity")
      dynamics_mode_ = DynamicsMode::GRAVITY;
    else
      RCLCPP_WARN(get_logger(), "Unknown dynamics_mode '%s'; using gravity.", mode.c_str());
  }

  if (info_.hardware_parameters.count("dynamics_urdf_path"))
    dynamics_urdf_path_ = trim_copy(info_.hardware_parameters.at("dynamics_urdf_path"));
  if (info_.hardware_parameters.count("dynamics_feedforward_source"))
    dynamics_feedforward_source_ =
      trim_copy(info_.hardware_parameters.at("dynamics_feedforward_source"));
  if (info_.hardware_parameters.count("dynamics_feedforward_topic"))
    dynamics_feedforward_topic_ =
      trim_copy(info_.hardware_parameters.at("dynamics_feedforward_topic"));
  if (info_.hardware_parameters.count("dynamics_topic_timeout_sec"))
    dynamics_topic_timeout_sec_ =
      std::max(0.01, std::stod(info_.hardware_parameters.at("dynamics_topic_timeout_sec")));
  if (info_.hardware_parameters.count("dynamics_use_commanded_position"))
    dynamics_use_commanded_position_ =
      string_to_bool(info_.hardware_parameters.at("dynamics_use_commanded_position"));
  if (info_.hardware_parameters.count("dynamics_torque_scale"))
    dynamics_torque_scale_ = std::stod(info_.hardware_parameters.at("dynamics_torque_scale"));
  if (info_.hardware_parameters.count("dynamics_torque_low_pass_alpha"))
    dynamics_torque_low_pass_alpha_ =
      std::stod(info_.hardware_parameters.at("dynamics_torque_low_pass_alpha"));
  dynamics_torque_low_pass_alpha_ = std::clamp(dynamics_torque_low_pass_alpha_, 0.0, 1.0);

  if (info_.hardware_parameters.count("dynamics_torque_limits_nm"))
  {
    const auto values = split_list(info_.hardware_parameters.at("dynamics_torque_limits_nm"));
    for (size_t i = 0; i < values.size() && i < dynamics_torque_limits_nm_.size(); ++i)
      dynamics_torque_limits_nm_[i] = std::max(0.0, std::stod(values[i]));
  }

  dynamics_torque_signs_ = arm_joint_signs_;
  if (info_.hardware_parameters.count("dynamics_torque_joint_signs"))
  {
    const auto signs = split_list(info_.hardware_parameters.at("dynamics_torque_joint_signs"));
    for (size_t i = 0; i < signs.size() && i < dynamics_torque_signs_.size(); ++i)
    {
      const double sign = std::stod(signs[i]);
      if (std::abs(sign) <= std::numeric_limits<double>::epsilon())
      {
        RCLCPP_WARN(get_logger(), "dynamics_torque_joint_signs[%zu] is zero; keeping %.1f.",
                    i, dynamics_torque_signs_[i]);
        continue;
      }
      dynamics_torque_signs_[i] = sign < 0.0 ? -1.0 : 1.0;
    }
  }

  if (enable_dynamics_feedforward_ &&
      arm_command_frame_format_ != ArmCommandFrameFormat::POSITION_TORQUE)
  {
    RCLCPP_WARN(
      get_logger(),
      "enable_dynamics_feedforward=true but arm_command_frame_format is legacy_position. "
      "Torque has nowhere to go, so dynamics feedforward is disabled.");
    enable_dynamics_feedforward_ = false;
  }

  if (dynamics_feedforward_source_ != "topic" &&
      dynamics_feedforward_source_ != "internal_pinocchio")
  {
    RCLCPP_WARN(
      get_logger(),
      "Unknown dynamics_feedforward_source '%s'; using topic.",
      dynamics_feedforward_source_.c_str());
    dynamics_feedforward_source_ = "topic";
  }
}

void MyArmHardware::initialize_dynamics_model()
{
  dynamics_model_ready_ = false;
  if (dynamics_feedforward_source_ != "internal_pinocchio")
    return;
#ifdef MY_ARM_HARDWARE_HAS_PINOCCHIO
  try
  {
    if (!dynamics_urdf_path_.empty())
      pinocchio::urdf::buildModel(dynamics_urdf_path_, dynamics_model_);
    else if (!info_.original_xml.empty())
      pinocchio::urdf::buildModelFromXML(info_.original_xml, dynamics_model_);
    else
    {
      RCLCPP_WARN(get_logger(), "No URDF XML/path available for Pinocchio dynamics.");
      return;
    }

    dynamics_data_ = std::make_unique<pinocchio::Data>(dynamics_model_);
    for (size_t i = 0; i < dynamics_joint_q_indices_.size(); ++i)
    {
      if (i >= info_.joints.size())
        break;
      const std::string & joint_name = info_.joints[i].name;
      if (!dynamics_model_.existJointName(joint_name))
      {
        RCLCPP_WARN(get_logger(), "Pinocchio model does not contain arm joint '%s'.", joint_name.c_str());
        return;
      }
      const auto joint_id = dynamics_model_.getJointId(joint_name);
      if (dynamics_model_.nqs[joint_id] != 1 || dynamics_model_.nvs[joint_id] != 1)
      {
        RCLCPP_WARN(
          get_logger(),
          "Pinocchio joint '%s' is not a 1-DoF joint (nq=%d, nv=%d).",
          joint_name.c_str(), dynamics_model_.nqs[joint_id], dynamics_model_.nvs[joint_id]);
        return;
      }
      dynamics_joint_q_indices_[i] = dynamics_model_.idx_qs[joint_id];
      dynamics_joint_v_indices_[i] = dynamics_model_.idx_vs[joint_id];
    }

    dynamics_model_ready_ = true;
    RCLCPP_WARN(
      get_logger(),
      "Pinocchio dynamics model loaded. Feedforward enabled=%s, frame_format=%s. "
      "Verify URDF inertials before applying nonzero torque on hardware.",
      enable_dynamics_feedforward_ ? "true" : "false",
      arm_command_frame_format_ == ArmCommandFrameFormat::POSITION_TORQUE ?
        "position_torque" : "legacy_position");
  }
  catch (const std::exception & e)
  {
    dynamics_model_ready_ = false;
    RCLCPP_ERROR(get_logger(), "Failed to initialize Pinocchio dynamics model: %s", e.what());
  }
#else
  if (enable_dynamics_feedforward_)
  {
    RCLCPP_ERROR(
      get_logger(),
      "Dynamics feedforward was requested, but my_arm_hardware was built without Pinocchio. "
      "Extended frames will carry zero torque until Pinocchio is installed and the package is rebuilt.");
  }
#endif
}

std::array<double, 6> MyArmHardware::compute_dynamics_torques(const rclcpp::Duration & period)
{
  (void)period;
  std::array<double, 6> tau_ros_nm{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  if (!enable_dynamics_feedforward_)
    return tau_ros_nm;

  if (dynamics_feedforward_source_ == "topic")
  {
    const double age =
      dynamics_topic_last_time_.nanoseconds() > 0 ?
      (get_clock()->now() - dynamics_topic_last_time_).seconds() :
      std::numeric_limits<double>::infinity();
    if (age > dynamics_topic_timeout_sec_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *clock_, 1000,
        "Dynamics torque topic '%s' is stale/unavailable (age %.3fs); sending zero torque.",
        dynamics_feedforward_topic_.c_str(), age);
      return tau_ros_nm;
    }
    for (size_t i = 0; i < tau_ros_nm.size(); ++i)
    {
      const double limit = dynamics_torque_limits_nm_[i];
      tau_ros_nm[i] = std::clamp(dynamics_topic_tau_ros_nm_[i], -limit, limit);
    }
    return tau_ros_nm;
  }

#ifdef MY_ARM_HARDWARE_HAS_PINOCCHIO
  if (!dynamics_model_ready_ || !dynamics_data_)
  {
    if (!dynamics_warned_unavailable_)
    {
      RCLCPP_WARN(get_logger(), "Dynamics feedforward requested but Pinocchio model is not ready; sending zero torque.");
      dynamics_warned_unavailable_ = true;
    }
    return tau_ros_nm;
  }

  try
  {
    const double dt = safe_period_seconds(period);
    Eigen::VectorXd q = pinocchio::neutral(dynamics_model_);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(dynamics_model_.nv);
    Eigen::VectorXd a = Eigen::VectorXd::Zero(dynamics_model_.nv);

    const size_t joints = std::min(arm_joint_count(), kArmFeedbackJointCount);
    for (size_t i = 0; i < joints; ++i)
    {
      const int q_index = dynamics_joint_q_indices_[i];
      const int v_index = dynamics_joint_v_indices_[i];
      if (q_index < 0 || v_index < 0)
        continue;

      const double position =
        dynamics_use_commanded_position_ && i < hw_commands_.size() && std::isfinite(hw_commands_[i]) ?
        hw_commands_[i] : hw_states_[i];
      q[q_index] = std::isfinite(position) ? position : 0.0;

      if (dynamics_mode_ == DynamicsMode::CORIOLIS)
      {
        const double velocity = i < hw_velocities_.size() && std::isfinite(hw_velocities_[i]) ?
          hw_velocities_[i] : 0.0;
        v[v_index] = velocity;
      }
      else if (dynamics_mode_ == DynamicsMode::FULL)
      {
        const double command = i < hw_commands_.size() && std::isfinite(hw_commands_[i]) ? hw_commands_[i] : q[q_index];
        if (!dynamics_command_history_ready_)
        {
          dynamics_last_commands_[i] = command;
          dynamics_last_command_velocities_[i] = 0.0;
        }
        const double cmd_velocity = (command - dynamics_last_commands_[i]) / dt;
        const double cmd_accel = (cmd_velocity - dynamics_last_command_velocities_[i]) / dt;
        v[v_index] = cmd_velocity;
        a[v_index] = cmd_accel;
        dynamics_last_commands_[i] = command;
        dynamics_last_command_velocities_[i] = cmd_velocity;
      }
    }
    dynamics_command_history_ready_ = true;

    const Eigen::VectorXd tau = pinocchio::rnea(dynamics_model_, *dynamics_data_, q, v, a);
    for (size_t i = 0; i < joints; ++i)
    {
      const int v_index = dynamics_joint_v_indices_[i];
      if (v_index < 0)
        continue;
      double target = tau[v_index] * dynamics_torque_scale_;
      if (!std::isfinite(target))
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *clock_, 1000,
          "Pinocchio returned non-finite torque for joint %zu; forcing zero.", i + 1);
        target = 0.0;
      }

      const double limit = dynamics_torque_limits_nm_[i];
      target = std::clamp(target, -limit, limit);
      if (dynamics_torque_low_pass_alpha_ <= 0.0)
        target = dynamics_last_tau_ros_nm_[i];
      else if (dynamics_torque_low_pass_alpha_ < 1.0)
        target = dynamics_last_tau_ros_nm_[i] +
          dynamics_torque_low_pass_alpha_ * (target - dynamics_last_tau_ros_nm_[i]);

      dynamics_last_tau_ros_nm_[i] = target;
      tau_ros_nm[i] = target;
    }
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR_THROTTLE(get_logger(), *clock_, 1000,
      "Pinocchio torque computation failed: %s. Sending zero torque.", e.what());
    tau_ros_nm.fill(0.0);
  }
#else
  if (!dynamics_warned_unavailable_)
  {
    RCLCPP_WARN(get_logger(), "Dynamics feedforward requested but Pinocchio support is not compiled in; sending zero torque.");
    dynamics_warned_unavailable_ = true;
  }
#endif
  return tau_ros_nm;
}

void MyArmHardware::publish_dynamics_torques(const std::array<double, 6> & tau_ros_nm)
{
  if (!dynamics_tau_pub_)
    return;
  std_msgs::msg::Float64MultiArray msg;
  msg.data.assign(tau_ros_nm.begin(), tau_ros_nm.end());
  dynamics_tau_pub_->publish(msg);
}

std::vector<uint8_t> MyArmHardware::build_position_command_frame()
{
  const size_t joints_to_send = std::min(arm_joint_count(), kArmFeedbackJointCount);
  std::vector<uint8_t> frame(kArmFrameLength, 0);
  frame[0] = kArmCommandHeader;
  for (size_t i = 0; i < joints_to_send; ++i)
  {
    const double sign = (i < arm_joint_signs_.size()) ? arm_joint_signs_[i] : 1.0;
    const double offset = (i < arm_joint_offsets_.size()) ? arm_joint_offsets_[i] : 0.0;
    long scaled = std::lround(sign * (hw_commands_[i] - offset) * pos_scale_);
    int16_t data16 = clamp_to_i16(scaled);
    frame[1 + 2 * i] = static_cast<uint8_t>(data16 & 0xFF);
    frame[1 + 2 * i + 1] = static_cast<uint8_t>((data16 >> 8) & 0xFF);
    if (i < last_sent_commands_.size())
      last_sent_commands_[i] = sign * static_cast<double>(data16) / pos_scale_ + offset;
  }
  for (size_t i = joints_to_send; i < last_sent_commands_.size() && i < hw_commands_.size(); ++i)
    last_sent_commands_[i] = hw_commands_[i];
  frame.back() = frame_checksum(frame);
  return frame;
}

std::vector<uint8_t> MyArmHardware::build_position_torque_command_frame(
  const std::array<double, 6> & tau_hw_nm)
{
  const size_t joints_to_send = std::min(arm_joint_count(), kArmFeedbackJointCount);
  std::vector<uint8_t> frame;
  frame.reserve(kArmPositionTorqueFrameLength);
  frame.push_back(kArmPositionTorqueCommandHeader);
  for (size_t i = 0; i < kArmFeedbackJointCount; ++i)
  {
    int16_t data16 = 0;
    if (i < joints_to_send)
    {
      const double sign = (i < arm_joint_signs_.size()) ? arm_joint_signs_[i] : 1.0;
      const double offset = (i < arm_joint_offsets_.size()) ? arm_joint_offsets_[i] : 0.0;
      long scaled = std::lround(sign * (hw_commands_[i] - offset) * pos_scale_);
      data16 = clamp_to_i16(scaled);
      if (i < last_sent_commands_.size())
        last_sent_commands_[i] = sign * static_cast<double>(data16) / pos_scale_ + offset;
    }
    append_i16_le(frame, data16);
  }
  for (size_t i = joints_to_send; i < last_sent_commands_.size() && i < hw_commands_.size(); ++i)
    last_sent_commands_[i] = hw_commands_[i];

  for (size_t i = 0; i < kArmFeedbackJointCount; ++i)
  {
    const float tau = static_cast<float>(std::isfinite(tau_hw_nm[i]) ? tau_hw_nm[i] : 0.0);
    append_f32_le(frame, tau);
  }
  frame.push_back(0);
  frame.back() = frame_checksum(frame);
  return frame;
}

void MyArmHardware::setup_homing_interface()
{
  if (homing_node_)
    return;

  homing_node_ = std::make_shared<rclcpp::Node>("arm_homing_bridge");
  auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

  homing_status_pub_ = homing_node_->create_publisher<std_msgs::msg::String>(
    kHomingStatusTopic, latched_qos);
  homing_state_pub_ = homing_node_->create_publisher<std_msgs::msg::UInt8>(
    kHomingStateTopic, latched_qos);
  dynamics_tau_pub_ = homing_node_->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/my_arm_system/dynamics_tau_ff_nm", rclcpp::QoS(10));
  dynamics_tau_sub_ = homing_node_->create_subscription<std_msgs::msg::Float64MultiArray>(
    dynamics_feedforward_topic_,
    rclcpp::QoS(10),
    [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
      const size_t count = std::min(msg->data.size(), dynamics_topic_tau_ros_nm_.size());
      for (size_t i = 0; i < count; ++i)
      {
        dynamics_topic_tau_ros_nm_[i] =
          std::isfinite(msg->data[i]) ? msg->data[i] : 0.0;
      }
      for (size_t i = count; i < dynamics_topic_tau_ros_nm_.size(); ++i)
        dynamics_topic_tau_ros_nm_[i] = 0.0;
      dynamics_topic_last_time_ = get_clock()->now();
    });

  homing_init_srv_ = homing_node_->create_service<std_srvs::srv::Trigger>(
    kHomingInitService,
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      std::string message;
      response->success = start_damiao_initialization("service", message);
      response->message = message;
    });

  homing_confirm_srv_ = homing_node_->create_service<std_srvs::srv::Trigger>(
    kHomingConfirmService,
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      std::string message;
      response->success = confirm_drop_pose("service", message);
      response->message = message;
    });

  homing_init_sub_ = homing_node_->create_subscription<std_msgs::msg::Empty>(
    kHomingInitTopic,
    rclcpp::QoS(1),
    [this](const std_msgs::msg::Empty::SharedPtr /*msg*/)
    {
      std::string message;
      const bool ok = start_damiao_initialization("topic", message);
      if (ok)
        RCLCPP_INFO(get_logger(), "%s", message.c_str());
      else
        RCLCPP_WARN(get_logger(), "%s", message.c_str());
    });

  homing_drop_pose_sub_ = homing_node_->create_subscription<std_msgs::msg::Empty>(
    kHomingConfirmTopic,
    rclcpp::QoS(1),
    [this](const std_msgs::msg::Empty::SharedPtr /*msg*/)
    {
      std::string message;
      const bool ok = confirm_drop_pose("topic", message);
      if (ok)
        RCLCPP_INFO(get_logger(), "%s", message.c_str());
      else
        RCLCPP_WARN(get_logger(), "%s", message.c_str());
    });

  homing_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  homing_executor_->add_node(homing_node_);
  homing_spin_thread_ = std::thread([executor = homing_executor_]() { executor->spin(); });
}

void MyArmHardware::teardown_homing_interface()
{
  if (homing_executor_)
    homing_executor_->cancel();
  if (homing_spin_thread_.joinable())
    homing_spin_thread_.join();
  if (homing_executor_ && homing_node_)
    homing_executor_->remove_node(homing_node_);
  homing_init_srv_.reset();
  homing_confirm_srv_.reset();
  homing_init_sub_.reset();
  homing_drop_pose_sub_.reset();
  homing_status_pub_.reset();
  homing_state_pub_.reset();
  dynamics_tau_pub_.reset();
  dynamics_tau_sub_.reset();
  homing_executor_.reset();
  homing_node_.reset();
}

void MyArmHardware::capture_post_homing_command_hold()
{
  std::lock_guard<std::mutex> lock(command_hold_mtx_);
  post_homing_hold_commands_ = hw_commands_;
  post_homing_command_hold_active_ = true;
  RCLCPP_WARN(
    get_logger(),
    "Captured command hold for STM32 zeroing window. rqt_joint_trajectory_controller should stay closed until FLAG_ALL_READY.");
}

void MyArmHardware::finish_post_homing_command_hold()
{
  std::lock_guard<std::mutex> lock(command_hold_mtx_);
  if (post_homing_hold_commands_.size() == hw_commands_.size())
    std::copy(post_homing_hold_commands_.begin(), post_homing_hold_commands_.end(), hw_commands_.begin());
  post_homing_command_hold_active_ = false;
  post_homing_hold_commands_.clear();
}

void MyArmHardware::enforce_post_homing_command_hold()
{
  std::lock_guard<std::mutex> lock(command_hold_mtx_);
  if (post_homing_command_hold_active_ &&
      post_homing_hold_commands_.size() == hw_commands_.size())
  {
    std::copy(post_homing_hold_commands_.begin(), post_homing_hold_commands_.end(), hw_commands_.begin());
  }
}

void MyArmHardware::publish_homing_state(
  RosMasterHomingState state,
  const std::string & message)
{
  {
    std::lock_guard<std::mutex> lock(homing_state_mtx_);
    homing_state_ = state;
  }

  if (homing_status_pub_)
  {
    std_msgs::msg::String status_msg;
    status_msg.data = message;
    homing_status_pub_->publish(status_msg);
  }
  if (homing_state_pub_)
  {
    std_msgs::msg::UInt8 state_msg;
    state_msg.data = static_cast<uint8_t>(state);
    homing_state_pub_->publish(state_msg);
  }
}

bool MyArmHardware::send_system_command(uint8_t command_code, std::string & failure_reason)
{
  if (!serial_ok_)
  {
    failure_reason = "Writer serial port is not available, so the homing command could not be sent.";
    return false;
  }

  try
  {
    auto frame = pack_system_command(command_code);
    std::scoped_lock lk(serial_mtx_);
    serial_.Write(frame);
    serial_.DrainWriteBuffer();
    return true;
  }
  catch (const std::exception & e)
  {
    failure_reason = std::string("Failed to send homing command to STM32: ") + e.what();
    serial_ok_ = false;
    return false;
  }
}

bool MyArmHardware::start_damiao_initialization(const std::string & source, std::string & message)
{
  RosMasterHomingState state;
  {
    std::lock_guard<std::mutex> lock(homing_state_mtx_);
    state = homing_state_;
  }

  if (state == RosMasterHomingState::WAITING_DAMIAO_READY)
  {
    message = "Damiao initialization command was already sent. Waiting for FLAG_DAMIAO_READY from STM32.";
    return false;
  }
  if (state == RosMasterHomingState::WAITING_OPERATOR_DROP_POSE)
  {
    message = "STM32 already reported FLAG_DAMIAO_READY. Move to the drop pose, close rqt_joint_trajectory_controller, then confirm drop pose.";
    return false;
  }
  if (state == RosMasterHomingState::WAITING_ALL_READY)
  {
    message = "Drop pose was already confirmed. Waiting for FLAG_ALL_READY from STM32.";
    return false;
  }
  if (state == RosMasterHomingState::ALL_READY)
  {
    message = "Homing is already complete. Normal operation is active.";
    return false;
  }

  std::string failure_reason;
  if (!send_system_command(kCmdStartDamiaoInit, failure_reason))
  {
    message = failure_reason;
    return false;
  }

  message =
    "FLAG_START_DAMIAO_INIT sent to STM32 from " + source +
    ". Waiting for STM32 to activate the Damiao motors and reply with FLAG_DAMIAO_READY.";
  publish_homing_state(RosMasterHomingState::WAITING_DAMIAO_READY, message);
  RCLCPP_WARN(get_logger(), "%s", message.c_str());
  return true;
}

bool MyArmHardware::confirm_drop_pose(const std::string & source, std::string & message)
{
  RosMasterHomingState state;
  {
    std::lock_guard<std::mutex> lock(homing_state_mtx_);
    state = homing_state_;
  }

  if (state == RosMasterHomingState::WAITING_INIT_COMMAND)
  {
    message = "Damiao initialization has not been started yet. Click Initialize Damiao or call /arm_homing/start_initialization first.";
    return false;
  }
  if (state == RosMasterHomingState::WAITING_DAMIAO_READY)
  {
    message = "STM32 has not reported FLAG_DAMIAO_READY yet. Wait for the ready prompt before confirming the drop pose.";
    return false;
  }
  if (state == RosMasterHomingState::WAITING_ALL_READY)
  {
    message = "FLAG_REACHED_DROP_POSE was already sent. Waiting for FLAG_ALL_READY from STM32.";
    return false;
  }
  if (state == RosMasterHomingState::ALL_READY)
  {
    message = "Homing is already complete. Normal operation is active.";
    return false;
  }

  std::string failure_reason;
  if (!send_system_command(kCmdReachedDropPose, failure_reason))
  {
    message = failure_reason;
    return false;
  }

  capture_post_homing_command_hold();

  message =
    "FLAG_REACHED_DROP_POSE sent to STM32 from " + source +
    ". The MCU should wait 2 seconds, call cm_motor_set_absolute_zero(), "
    "then reply with FLAG_ALL_READY. Keep rqt_joint_trajectory_controller closed until homing is complete.";
  publish_homing_state(RosMasterHomingState::WAITING_ALL_READY, message);
  RCLCPP_WARN(get_logger(), "%s", message.c_str());
  return true;
}

bool MyArmHardware::can_write_motion_commands()
{
  std::lock_guard<std::mutex> lock(homing_state_mtx_);
  return homing_state_ == RosMasterHomingState::WAITING_OPERATOR_DROP_POSE ||
         homing_state_ == RosMasterHomingState::ALL_READY;
}

void MyArmHardware::handle_system_event(uint8_t event_code)
{
  if (!first_power_on_)
  {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *clock_, 2000,
      "Ignoring STM32 homing system event 0x%02X because first_power_on is false.",
      event_code);
    return;
  }

  switch (event_code)
  {
    case kEventDamiaoReady:
      publish_homing_state(
        RosMasterHomingState::WAITING_OPERATOR_DROP_POSE,
        "STM32 sent FLAG_DAMIAO_READY. Move the arm to the drop pose with RQT if needed, "
        "then close rqt_joint_trajectory_controller before clicking Confirm Drop Pose, run "
        "`ros2 service call /arm_homing/confirm_drop_pose std_srvs/srv/Trigger {}` "
        "or publish std_msgs/msg/Empty to /arm_homing/reached_drop_pose.");
      RCLCPP_WARN(
        get_logger(),
        "STM32 homing handshake: FLAG_DAMIAO_READY received. Move the arm to the drop pose, "
        "close rqt_joint_trajectory_controller, then call `%s` or publish to `%s`.",
        kHomingConfirmService,
        kHomingConfirmTopic);
      break;

    case kEventAllReady:
      finish_post_homing_command_hold();
      last_sent_commands_ = hw_commands_;
      sync_commands_on_next_feedback_ = false;
      publish_homing_state(
        RosMasterHomingState::ALL_READY,
        "STM32 sent FLAG_ALL_READY. Homing is complete. Reopen rqt_joint_trajectory_controller if you need more manual motion.");
      RCLCPP_WARN(
        get_logger(),
        "STM32 homing handshake: FLAG_ALL_READY received. Homing is complete.");
      break;

    default:
      RCLCPP_INFO_THROTTLE(
        get_logger(), *clock_, 2000,
        "Ignoring unknown STM32 system event 0x%02X.", event_code);
      break;
  }
}

}  // namespace my_arm_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  my_arm_hardware::MyArmHardware, hardware_interface::SystemInterface)
