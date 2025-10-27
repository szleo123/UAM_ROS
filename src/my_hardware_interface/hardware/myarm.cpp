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

#include <chrono>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

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
      if (buf.size() - i < frame_len) break;
      if (buf.size() - i < 3) break;
      // checksum over bytes starting at [len]
      uint32_t s = 0; for (size_t k = i+2; k < i+frame_len-1; ++k) s += buf[k];
      uint8_t chk = buf[i + frame_len - 1];
      if (chk != static_cast<uint8_t>(s & 0xFF)) { ++i; continue; }
      if (buf[i+4] != 0x21 || buf[i+5] != 0x37) { ++i; continue; } // not a state frame

      current_units = static_cast<int16_t>(buf[i+14] | buf[i+15] << 8);
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

namespace my_arm_hardware
{
bool MyArmHardware::wait_for_initial_feedback(std::array<double,6>& q, double timeout_sec)
{
  constexpr uint8_t HEADER = 0xAA;
  constexpr size_t FRAME_LEN = 14;
  constexpr size_t N = 6;

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
    while (buf_local.size() - i >= FRAME_LEN)
    {
      if (buf_local[i] != HEADER) { ++i; continue; }
      if (buf_local.size() - i < FRAME_LEN) break;

      uint32_t sum = 0;
      for (size_t k = 0; k < FRAME_LEN - 1; ++k) sum += buf_local[i + k];
      uint8_t cs = static_cast<uint8_t>(sum & 0xFF);
      uint8_t rxcs = buf_local[i + FRAME_LEN - 1];

      if (cs == rxcs)
      {
        // decode 6 int16 LE -> double
        for (size_t j = 0; j < N; ++j)
        {
          uint8_t lo = buf_local[i + 1 + 2*j];
          uint8_t hi = buf_local[i + 1 + 2*j + 1];
          int16_t raw = static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) | lo);
          q[j] = static_cast<double>(raw) / pos_scale_;
        }
        return true;
      }
      ++i; // bad checksum: slide window
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

  if (info_.hardware_parameters.count("hw_slowdown"))
    hw_slowdown_ = std::stod(info_.hardware_parameters.at("hw_slowdown"));
  else
    hw_slowdown_ = 1.0;

  if (info_.hardware_parameters.count("initial_read_timeout_sec"))
    initial_read_timeout_sec_ = std::stod(info_.hardware_parameters.at("initial_read_timeout_sec"));

  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

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
              "Initialized MyArmSystem with %i joints, writer_port=%s, reader_port=%s, baud=%u, scale=%.1f, slowdown=%.1f",
              static_cast<int>(info_.joints.size()), serial_port_path_.c_str(), reader_port_path_.c_str(), baudrate_, pos_scale_, hw_slowdown_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MyArmHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring ...please wait...");
  try 
  {
    std::scoped_lock lk(serial_mtx_);

    if (serial_.IsOpen())
      serial_.Close();

    serial_.Open(serial_port_path_);
    serial_.SetBaudRate(baud_from_uint(baudrate_));
    serial_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    serial_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
    serial_.SetParity(LibSerial::Parity::PARITY_NONE);
    serial_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);

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
    if (reader_.IsOpen())
      reader_.Close();

    reader_.Open(reader_port_path_);
    reader_.SetBaudRate(baud_from_uint(baudrate_));
    reader_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    reader_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
    reader_.SetParity(LibSerial::Parity::PARITY_NONE);
    reader_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);

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

    if (gripper_.IsOpen())
      gripper_.Close();

    gripper_.Open(gripper_port_path_);
    gripper_.SetBaudRate(baud_from_uint(gripper_baudrate_));
    gripper_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    gripper_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
    gripper_.SetParity(LibSerial::Parity::PARITY_NONE);
    gripper_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);

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

  // Reset states/commands 
  for (size_t i = 0; i < hw_states_.size(); i++)
  {
    hw_states_[i] = 0.0;
    hw_commands_[i] = 0.0;
    hw_velocities_[i] = 0.0;
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
    for (size_t i = 0; i < hw_states_.size(); ++i) {
      // keep whatever you had (0 or previous), but align commands to states to avoid jumps
      hw_commands_[i] = hw_states_[i];
    }
    RCLCPP_ERROR(get_logger(),
      "Initial feedback NOT received within %.2fs. write() will be skipped until feedback arrives.",
      initial_read_timeout_sec_);
  }

  // Set gripper to open 
  hw_commands_[6] = 0.0;  
  hw_states_[6] = 0.0;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MyArmHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");

  // Freeze commands at current positions
  for (size_t i = 0; i < hw_states_.size(); i++)
  {
    hw_commands_[i] = hw_states_[i];
  }
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
    for (size_t i = 0; i < hw_states_.size(); ++i)
    {
      double prev = hw_states_[i];
      hw_states_[i] = hw_commands_[i];  // snap to command for perfect sim
      hw_velocities_[i] = (hw_states_[i] - prev) / period.seconds(); // calculate velocity
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
    constexpr uint8_t HEADER = 0xAA;
    constexpr size_t FRAME_LEN = 14; 
    constexpr size_t N = 6; // number of joints expected

    size_t i = 0; 
    while (rx_buffer_.size() - i >= FRAME_LEN) {
      // Find header 
      if (rx_buffer_[i] != HEADER) {
        i++;
        continue;
      }

      // We have a candidate header at i; ensure we have full frame 
      if (rx_buffer_.size() - i < FRAME_LEN) {
        break; // wait for more data
      }

      // Compute checksum over bytes [i .. i+12]
      uint32_t sum = 0; 
      for (size_t k = 0; k < FRAME_LEN -1; k++) {
        sum += rx_buffer_[i + k];
      }
      uint8_t checksum = static_cast<uint8_t>(sum & 0xFF);
      uint8_t rx_checksum = rx_buffer_[i + FRAME_LEN -1];

      if (checksum == rx_checksum) {
        // Valid frame: decode 6 little-endian int16 angles 
        for (size_t j = 0; j < N; j++) {
          uint8_t lo = rx_buffer_[i + 1 + 2*j];
          uint8_t hi = rx_buffer_[i + 1 + 2*j + 1];
          int16_t raw = static_cast<int16_t>(static_cast<uint16_t>(hi) << 8 | lo);
          double angle = static_cast<double>(raw) / pos_scale_;
          if (j < hw_states_.size()) {
            double prev = hw_states_[j];
            hw_states_[j] = angle;
            hw_velocities_[j] = (hw_states_[j] - prev) / period.seconds(); // calculate velocity
          }
        }

        i += FRAME_LEN; // advance past this frame

        // if this is the first valid feedback, mark it
        last_feedback_time_ = get_clock()->now();
        if (!initial_positions_received_) {
          for (size_t j = 0; j < hw_states_.size(); j++) {
            hw_commands_[j] = hw_states_[j]; // align commands to states
          }
          initial_positions_received_ = true;
          RCLCPP_WARN(get_logger(),
            "Initial positions synced from hardware. Writes now enabled.");
        }
        // optionally log decoded values occasionally 
        RCLCPP_INFO_THROTTLE(get_logger(), *clock_, 1000, 
          "Read positions: %.3f %.3f %.3f %.3f %.3f %.3f",
          hw_states_[0], hw_states_[1], hw_states_[2],
          hw_states_[3], hw_states_[4], hw_states_[5]);

        
      } else {
        // Bad checksum; skip this header 
        i++;
      }
    }

    // Erase all processed bytes
    if (i > 0) {
      rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + static_cast<long>(i));
    }
  }

  if (gripper_ok_){
    double prev = hw_states_[6];
    hw_states_[6] = hw_commands_[6];
    hw_velocities_[6] = (hw_states_[6] - prev) / period.seconds();
  }
  // if (gripper_ok_) {
  //   try {
  //     std::scoped_lock gk(gripper_mtx_);
  //     // pull in all available bytes (non-blocking)
  //     char c = 0;
  //     while (gripper_.IsDataAvailable()) {
  //       gripper_.ReadByte(c, 0);
  //       gripper_rx_.push_back(static_cast<uint8_t>(c));
  //     }

  //     // parse as many valid frames as possible
  //     bool got_any = false;
  //     int16_t cur_units = 0;
  //     while (gripper_try_parse_state(gripper_rx_, cur_units)) {
  //       got_any = true;
  //       double ros_pos = units_to_grip(cur_units);
  //       double prev = hw_states_[6];
  //       hw_states_[6] = ros_pos;
  //       hw_velocities_[6] = (hw_states_[6] - prev) / period.seconds();
  //       // (Optional)
  //       RCLCPP_INFO_THROTTLE(get_logger(), *clock_, 2000,
  //         "Gripper read: units %d -> ROS %.3f", (int)cur_units, ros_pos);
  //     }

  //     if (got_any){
  //       grip_waiting_ = false;
  //     } else if (grip_waiting_ && get_clock()->now() > grip_deadline_) {
  //       // timeout waiting for reply
  //       grip_waiting_ = false;
  //       RCLCPP_WARN_THROTTLE(get_logger(), *clock_, 2000,
  //         "Gripper read: reply timeout.");
  //     }
  //   } catch (const std::exception& e) {
  //     gripper_ok_ = false;
  //     RCLCPP_ERROR_THROTTLE(get_logger(), *clock_, 2000,
  //       "Gripper read failed: %s", e.what());
  //   }
  // }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MyArmHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!serial_ok_ && !gripper_ok_)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *clock_, 2000, "Serial not OK; skipping write().");
    return hardware_interface::return_type::OK;
  }

  // ---- Gripper TX (full-duplex same port) ----
  if (gripper_ok_) {
    const auto now = get_clock()->now();
    const double ros_cmd = hw_commands_[6];              // ROS units (e.g., radians)
      uint16_t tgt = grip_to_units(ros_cmd);
      try {
        std::scoped_lock gk(gripper_mtx_);
        auto f = gripper_pack_target(gripper_id_, tgt);
        gripper_.Write(f);
        gripper_.DrainWriteBuffer();
        // try {
        //   gripper_.DrainWriteBuffer();
        //   auto t0 = get_clock()->now();
        //   while ((get_clock()->now() - t0).nanoseconds() < 5000000) { // 5ms wait
        //     char c= 0; 
        //     while (gripper_.IsDataAvailable()) {
        //       gripper_.ReadByte(c, 0);
        //       gripper_rx_.push_back(static_cast<uint8_t>(c));
        //     }
        //     int16_t cur_units = 0;
        //     if (gripper_try_parse_state(gripper_rx_, cur_units)) {
        //       RCLCPP_WARN_THROTTLE(get_logger(), *clock_, 2000, 
        //         "Gripper immediate read after write succeeded.");
        //       double ros_pos = units_to_grip(cur_units);
        //       hw_states_[6] = ros_pos;
        //       break; 
        //     }
        //   }
        //   rclcpp::sleep_for(std::chrono::microseconds(100));

        // } catch (const std::exception& e) {
        //   RCLCPP_WARN_THROTTLE(get_logger(), *clock_, 2000,
        //     "Gripper immediate read after write failed: %s", e.what());
        // }

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
      RCLCPP_WARN_THROTTLE(get_logger(), *clock_, 2000,
        "Initial positions not yet received; skipping write().");
      return hardware_interface::return_type::OK;
    }
    // Build frame: 0xFF + 6*int16(le) + checksum
    // If URDF lists more than 6 joints we’ll send first 6; fewer => pad zeros.
    const size_t N = 6;
    std::vector<uint8_t> frame(14, 0); // 1+12+1
    frame[0] = 0xFF;  // header
    for (size_t i = 0; i < N; ++i)
    {
      double pos = (i < hw_commands_.size()) ? hw_commands_[i] : 0.0;
      long scaled = std::lround(pos * pos_scale_);
      int16_t data16 = clamp_to_i16(scaled);
      frame[1 + 2 * i] = static_cast<uint8_t>(data16 & 0xFF);         // LSB
      frame[1 + 2 * i + 1] = static_cast<uint8_t>((data16 >> 8) & 0xFF); // MSB
    }


    // checksum = sum(header + payload) & 0xFF
    uint8_t sum = 0;
    for (size_t i = 0; i < 13; i++) {
      sum += frame[i];
    }
    frame[13] = sum;

    try
    {
      std::scoped_lock lk(serial_mtx_);
      serial_.Write(frame);
      // RCLCPP_INFO_THROTTLE(get_logger(), *clock_, 1000, "write() OK");
      RCLCPP_INFO_THROTTLE(get_logger(), *clock_, 1000, 
          "Write positions: %.3f %.3f %.3f %.3f %.3f %.3f",
          hw_commands_[0], hw_commands_[1], hw_commands_[2],
          hw_commands_[3], hw_commands_[4], hw_commands_[5]);
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

  // Close writer port
  try {
    std::scoped_lock lk(serial_mtx_);
    if (serial_.IsOpen()) {
      serial_.FlushIOBuffers();          // optional: discard pending I/O
      serial_.Close();
    }
    serial_ok_ = false;
  } catch (const std::exception &e) {
    RCLCPP_WARN(get_logger(), "Error while closing serial writer: %s", e.what());
  }

  // Close reader port (if you use a dedicated one)
  try {
    std::scoped_lock rk(reader_mtx_);
    if (reader_.IsOpen()) {
      reader_.FlushIOBuffers();
      reader_.Close();
    }
    reader_ok_ = false;
  } catch (const std::exception &e) {
    RCLCPP_WARN(get_logger(), "Error while closing serial reader: %s", e.what());
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

}  // namespace my_arm_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  my_arm_hardware::MyArmHardware, hardware_interface::SystemInterface)
