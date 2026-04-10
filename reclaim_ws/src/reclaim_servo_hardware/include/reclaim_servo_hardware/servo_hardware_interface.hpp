#ifndef RECLAIM_SERVO_HARDWARE__SERVO_HARDWARE_INTERFACE_HPP_
#define RECLAIM_SERVO_HARDWARE__SERVO_HARDWARE_INTERFACE_HPP_

#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

using hardware_interface::return_type;
using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace reclaim_servo_hardware
{

struct ServoCalibration
{
  double zero_us;   // microseconds at 0 radians (straight up)
  double scale;     // rad/µs conversion factor
  bool inverted;    // negate radians before conversion
  double min_us;    // hard limit low (µs)
  double max_us;    // hard limit high (µs)
};

class ServoHardwareInterface : public hardware_interface::SystemInterface
{
public:
  // Lifecycle
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

  // Interfaces
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // Control loop (called at update_rate Hz)
  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Serial port
  int serial_fd_ = -1;
  std::string serial_port_;
  int baud_rate_;

  // Per-joint data (4 joints: J1-J4)
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
  std::vector<ServoCalibration> calibrations_;
  std::vector<double> prev_commands_;

  // Fixed joint µs value (not in MoveIt2 chain, not in ros2_control)
  static constexpr int J5_FIXED_US = 701;  // wrist rotate (DS3218, -98 deg)
  // J6 gripper is now ros2_control joint index 4 (no longer fixed)

  // Helpers
  int radians_to_us(double radians, const ServoCalibration & cal) const;
  bool open_serial();
  void close_serial();
  bool send_command(const std::string & cmd);
};

}  // namespace reclaim_servo_hardware

#endif  // RECLAIM_SERVO_HARDWARE__SERVO_HARDWARE_INTERFACE_HPP_
