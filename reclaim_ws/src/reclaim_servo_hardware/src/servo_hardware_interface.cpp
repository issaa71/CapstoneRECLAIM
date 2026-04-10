#include "reclaim_servo_hardware/servo_hardware_interface.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <sstream>

// Linux serial
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "pluginlib/class_list_macros.hpp"

namespace reclaim_servo_hardware
{

// ---------------------------------------------------------------
// on_init: parse URDF ros2_control parameters
// ---------------------------------------------------------------
CallbackReturn ServoHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // Hardware-level parameters
  serial_port_ = info_.hardware_parameters.at("serial_port");
  baud_rate_ = std::stoi(info_.hardware_parameters.at("baud_rate"));

  const size_t n = info_.joints.size();
  hw_commands_.resize(n, 0.0);
  hw_states_.resize(n, 0.0);
  prev_commands_.resize(n, std::numeric_limits<double>::quiet_NaN());
  calibrations_.resize(n);

  // Read per-joint calibration from URDF <param> tags
  for (size_t i = 0; i < n; i++)
  {
    const auto & joint = info_.joints[i];

    calibrations_[i].zero_us  = std::stod(joint.parameters.at("zero_us"));
    calibrations_[i].scale    = std::stod(joint.parameters.at("scale"));
    calibrations_[i].inverted = (joint.parameters.at("inverted") == "true");
    calibrations_[i].min_us   = std::stod(joint.parameters.at("min_us"));
    calibrations_[i].max_us   = std::stod(joint.parameters.at("max_us"));

    RCLCPP_INFO(rclcpp::get_logger("ServoHardwareInterface"),
      "Joint[%zu] '%s': zero=%.0f scale=%.6f inv=%s range=[%.0f, %.0f]",
      i, joint.name.c_str(),
      calibrations_[i].zero_us, calibrations_[i].scale,
      calibrations_[i].inverted ? "true" : "false",
      calibrations_[i].min_us, calibrations_[i].max_us);
  }

  RCLCPP_INFO(rclcpp::get_logger("ServoHardwareInterface"),
    "Initialized %zu joints. Serial: %s @ %d baud",
    n, serial_port_.c_str(), baud_rate_);

  return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------
// Serial helpers (POSIX termios, no external dependencies)
// ---------------------------------------------------------------
bool ServoHardwareInterface::open_serial()
{
  serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (serial_fd_ < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("ServoHardwareInterface"),
      "Cannot open '%s': %s", serial_port_.c_str(), strerror(errno));
    return false;
  }

  struct termios tty;
  memset(&tty, 0, sizeof(tty));
  if (tcgetattr(serial_fd_, &tty) != 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("ServoHardwareInterface"),
      "tcgetattr failed: %s", strerror(errno));
    close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }

  // Baud rate
  speed_t baud = B115200;
  if (baud_rate_ == 9600) baud = B9600;
  else if (baud_rate_ == 57600) baud = B57600;
  cfsetospeed(&tty, baud);
  cfsetispeed(&tty, baud);

  // 8N1, no flow control
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
  tty.c_cflag |= (CLOCAL | CREAD);

  // Raw mode
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP |
                    INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY);
  tty.c_oflag &= ~(OPOST | ONLCR);
  tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

  // Non-blocking read with short timeout
  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 1;  // 0.1s

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("ServoHardwareInterface"),
      "tcsetattr failed: %s", strerror(errno));
    close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }

  tcflush(serial_fd_, TCIOFLUSH);

  RCLCPP_INFO(rclcpp::get_logger("ServoHardwareInterface"),
    "Serial port '%s' opened @ %d baud", serial_port_.c_str(), baud_rate_);
  return true;
}

void ServoHardwareInterface::close_serial()
{
  if (serial_fd_ >= 0)
  {
    close(serial_fd_);
    serial_fd_ = -1;
    RCLCPP_INFO(rclcpp::get_logger("ServoHardwareInterface"), "Serial port closed");
  }
}

bool ServoHardwareInterface::send_command(const std::string & cmd)
{
  if (serial_fd_ < 0) return false;
  ssize_t n = ::write(serial_fd_, cmd.c_str(), cmd.size());
  return (n == static_cast<ssize_t>(cmd.size()));
}

// ---------------------------------------------------------------
// Radians -> microseconds (same formula for all joints)
// us = zero_us + (radians / scale)
// Inverted: us = zero_us - (radians / scale)
// Then clamp to [min_us, max_us]
// ---------------------------------------------------------------
int ServoHardwareInterface::radians_to_us(
  double radians, const ServoCalibration & cal) const
{
  double r = cal.inverted ? -radians : radians;
  double us = cal.zero_us + (r / cal.scale);
  us = std::clamp(us, cal.min_us, cal.max_us);
  return static_cast<int>(std::round(us));
}

// ---------------------------------------------------------------
// Lifecycle: on_configure — open serial port
// ---------------------------------------------------------------
CallbackReturn ServoHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ServoHardwareInterface"), "Configuring...");
  if (!open_serial())
  {
    return CallbackReturn::ERROR;
  }

  // Wait for Teensy to boot after serial open
  usleep(2000000);  // 2 seconds

  // Send ARM command to initialize servos (attaches PWM pins, goes to firmware home)
  RCLCPP_INFO(rclcpp::get_logger("ServoHardwareInterface"), "Sending ARM command...");
  send_command("ARM\n");
  usleep(2000000);  // Wait for arm to reach firmware home

  // Smoothly move from firmware home to SEARCH pose over 2.5 seconds
  // using firmware's quintic trajectory (T parameter). This prevents a violent snap.
  // Search pose radians: [0.0, 1.263, 1.354, -1.931]
  // Computed µs: J1=1229, J2=2209, J3=1895, J4=1410
  // After this, hw_commands_ must match the search pose radians.
  //
  // OLD (straight up — zero_us values = 0 rad for all joints):
  // std::ostringstream zero_cmd;
  // zero_cmd << "SETUS";
  // for (size_t i = 0; i < calibrations_.size() && i < 4; i++)
  //   zero_cmd << " " << static_cast<int>(calibrations_[i].zero_us);
  // zero_cmd << " " << J5_FIXED_US;
  // if (calibrations_.size() > 4)
  //   zero_cmd << " " << static_cast<int>(calibrations_[4].zero_us);
  // else
  //   zero_cmd << " 1400";
  // zero_cmd << " T 2500\n";

  // Search pose µs (from URDF calibration: us = zero_us + rad/scale)
  const int search_us[4] = {1229, 2209, 1895, 1410};
  const double search_rad[4] = {0.0, 1.263, 1.354, -1.931};

  std::ostringstream startup_cmd;
  startup_cmd << "SETUS";
  for (int i = 0; i < 4; i++)
    startup_cmd << " " << search_us[i];
  startup_cmd << " " << J5_FIXED_US;
  // J6 gripper — start fully open
  if (calibrations_.size() > 4)
    startup_cmd << " " << static_cast<int>(calibrations_[4].zero_us);
  else
    startup_cmd << " 1400";
  startup_cmd << " T 2500\n";

  RCLCPP_INFO(rclcpp::get_logger("ServoHardwareInterface"),
    "Moving to search pose (2.5s trajectory): %s", startup_cmd.str().c_str());
  send_command(startup_cmd.str());
  usleep(3000000);  // Wait for 2.5s trajectory + 0.5s buffer

  // Set hw_commands_ and hw_states_ to match search pose
  // so ros2_control knows where the arm actually is
  for (int i = 0; i < 4 && i < static_cast<int>(hw_commands_.size()); i++)
  {
    hw_commands_[i] = search_rad[i];
    hw_states_[i] = search_rad[i];
  }

  return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------
// Lifecycle: on_activate — begin accepting commands
// ---------------------------------------------------------------
CallbackReturn ServoHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ServoHardwareInterface"), "Activating...");

  // Force first write by invalidating previous commands
  for (size_t i = 0; i < prev_commands_.size(); i++)
  {
    prev_commands_[i] = std::numeric_limits<double>::quiet_NaN();
  }

  return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------
// Lifecycle: on_deactivate
// ---------------------------------------------------------------
CallbackReturn ServoHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ServoHardwareInterface"), "Deactivating...");
  return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------
// Lifecycle: on_cleanup — close serial
// ---------------------------------------------------------------
CallbackReturn ServoHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ServoHardwareInterface"), "Cleaning up...");
  send_command("DISARM\n");
  close_serial();
  return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------
// Lifecycle: on_shutdown — close serial
// ---------------------------------------------------------------
CallbackReturn ServoHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ServoHardwareInterface"), "Shutting down...");
  send_command("DISARM\n");
  close_serial();
  return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------
// export_state_interfaces — position only
// ---------------------------------------------------------------
std::vector<hardware_interface::StateInterface>
ServoHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_states_[i]);
  }
  return interfaces;
}

// ---------------------------------------------------------------
// export_command_interfaces — position only
// ---------------------------------------------------------------
std::vector<hardware_interface::CommandInterface>
ServoHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_commands_[i]);
  }
  return interfaces;
}

// ---------------------------------------------------------------
// read() — open-loop: echo commanded positions as state
// ---------------------------------------------------------------
return_type ServoHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (size_t i = 0; i < hw_states_.size(); i++)
  {
    hw_states_[i] = hw_commands_[i];
  }
  return return_type::OK;
}

// ---------------------------------------------------------------
// write() — convert 4 joint radians to 6 µs, send SETUS
// Only sends when commands change (avoids flooding serial)
// ---------------------------------------------------------------
return_type ServoHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Check if any command changed
  bool changed = false;
  for (size_t i = 0; i < hw_commands_.size(); i++)
  {
    if (std::isnan(prev_commands_[i]) ||
        std::abs(hw_commands_[i] - prev_commands_[i]) > 1e-6)
    {
      changed = true;
      break;
    }
  }

  if (!changed) return return_type::OK;

  // Safety: reject NaN commands (std::clamp with NaN is undefined behavior)
  for (size_t i = 0; i < hw_commands_.size(); i++)
  {
    if (std::isnan(hw_commands_[i]))
    {
      RCLCPP_WARN(rclcpp::get_logger("ServoHardwareInterface"),
        "NaN detected in hw_commands_[%zu], skipping write", i);
      return return_type::OK;
    }
  }

  // Convert 4 arm joints to µs
  std::vector<int> us(6);
  for (size_t i = 0; i < hw_commands_.size() && i < 4; i++)
  {
    us[i] = radians_to_us(hw_commands_[i], calibrations_[i]);
  }
  // J5 wrist rotate — fixed (not in ros2_control)
  us[4] = J5_FIXED_US;
  // J6 gripper — ros2_control index 4
  if (hw_commands_.size() > 4 && calibrations_.size() > 4)
    us[5] = radians_to_us(hw_commands_[4], calibrations_[4]);
  else
    us[5] = 1400;  // default open

  // Build SETUS command: "SETUS 1229 1673 1320 2230 1755 1500\n"
  std::ostringstream oss;
  oss << "SETUS";
  for (int i = 0; i < 6; i++)
  {
    oss << " " << us[i];
  }
  oss << "\n";

  if (!send_command(oss.str()))
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("ServoHardwareInterface"),
      "Failed to send SETUS command");
    return return_type::ERROR;
  }

  prev_commands_ = hw_commands_;
  return return_type::OK;
}

}  // namespace reclaim_servo_hardware

// ---------------------------------------------------------------
// Plugin registration
// ---------------------------------------------------------------
PLUGINLIB_EXPORT_CLASS(
  reclaim_servo_hardware::ServoHardwareInterface,
  hardware_interface::SystemInterface)
