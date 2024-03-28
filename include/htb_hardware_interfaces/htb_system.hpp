#ifndef HTB_HARDWARE_INTERFACES__HTB_SYSTEM_HPP_
#define HTB_HARDWARE_INTERFACES__HTB_SYSTEM_HPP_

#include "htb_hardware_interfaces/visibility_control.hpp"

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/rclcpp.hpp"

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"

#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"

#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "htb_hardware_interfaces/wheel.hpp"
#include "hoverboard_driver/protocol.hpp"
#include "hoverboard_driver/pid.hpp"
#include "hoverboard_driver/config.hpp"

namespace htb_hardware_interfaces
{
using return_type = hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using StateInterface = hardware_interface::StateInterface;
using CommandInterface = hardware_interface::CommandInterface;

using JointState = sensor_msgs::msg::JointState;
using Float32MultiArray = std_msgs::msg::Float32MultiArray;

class HtbSystem : public hardware_interface::SystemInterface
{

struct Config
{
  std::string left_wheel_name = "";
  std::string right_wheel_name = "";
  float loop_rate = 0.0;
  std::string device = "";
  int baud_rate = 0;
  int timeout_ms = 0;
  int enc_counts_per_rev = 0;
  int pid_p = 0;
  int pid_d = 0;
  int pid_i = 0;
  int pid_o = 0;
};

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(HtbSystem)

  HTB_HARDWARE_INTERFACES_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo& hardware_info) override;

  HTB_HARDWARE_INTERFACES_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  HTB_HARDWARE_INTERFACES_PUBLIC
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

  HTB_HARDWARE_INTERFACES_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  HTB_HARDWARE_INTERFACES_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  HTB_HARDWARE_INTERFACES_PUBLIC
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

  HTB_HARDWARE_INTERFACES_PUBLIC
  CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;

  HTB_HARDWARE_INTERFACES_PUBLIC
  std::vector<StateInterface> export_state_interfaces() override;

  HTB_HARDWARE_INTERFACES_PUBLIC
  std::vector<CommandInterface> export_command_interfaces() override;

  HTB_HARDWARE_INTERFACES_PUBLIC
  return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  HTB_HARDWARE_INTERFACES_PUBLIC
  return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

protected:
  void cleanup_node();
  void protocol_recv (char);
  void on_encoder_update (int16_t right, int16_t left);

  Config cfg_;
  Wheel wheel_l_;
  Wheel wheel_r_;

  // Hoverboard driver

  double wheel_radius_;
  double max_velocity_ = 0.0;
  int direction_correction_ = 1;
  int inverted_ = 1;
  std::string port_;

  rclcpp::Time last_read_;
  // Last known encoder values
  int16_t last_wheelcountR_;
  int16_t last_wheelcountL_;
  // Count of full encoder wraps
  int multR_;
  int multL_;
  // Thresholds for calculating the wrap
  int low_wrap_;
  int high_wrap_;

  // Hoverboard protocol
  int port_fd_;
  int msg_len_ = 0;
  char prev_byte_ = 0;
  uint16_t start_frame_ = 0;
  char* p_;
  SerialFeedback msg_, prev_msg_;

  // PID pids[2];

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::executors::MultiThreadedExecutor executor_;
  std::unique_ptr<std::thread> executor_thread_;

  std::vector<std::string> velocity_command_joint_order_;


};

}  // namespace htb_hardware_interfaces

#endif  // HTB_HARDWARE_INTERFACES__HTB_SYSTEM_HPP_
