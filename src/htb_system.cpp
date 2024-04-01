#include "htb_hardware_interfaces/htb_system.hpp"

#include <string>
#include <vector>
#include <algorithm>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include "rclcpp/logging.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"


// NOTES:
// - check: 
namespace htb_hardware_interfaces
{
CallbackReturn HtbSystem::on_init(const hardware_interface::HardwareInfo& hardware_info)
{
  // TODO: Initialize drivers
  RCLCPP_INFO(rclcpp::get_logger("HtbSystem"), "Initializing");

  if (hardware_interface::SystemInterface::on_init(hardware_info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("HtbSystem"), "Initializing");

  // Get joint names from list of joints
  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];

  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
  cfg_.inverted = std::stoi(info_.hardware_parameters["inverted"]);

  // Check if device is a serial port
  RCLCPP_INFO(rclcpp::get_logger("HtbSystem"), "Configuring serial port: %s", cfg_.device.c_str());
  port_ = cfg_.device;

  RCLCPP_INFO(rclcpp::get_logger("HtbSystem"), "Configuring PID values.");
  if (info_.hardware_parameters.count("pid_p") > 0)
  {
    cfg_.pid_p = std::stoi(info_.hardware_parameters["pid_p"]);
    cfg_.pid_d = std::stoi(info_.hardware_parameters["pid_d"]);
    cfg_.pid_i = std::stoi(info_.hardware_parameters["pid_i"]);
    cfg_.pid_o = std::stoi(info_.hardware_parameters["pid_o"]);
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "PID values not supplied, using defaults.");
  }


  wheel_l_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
  wheel_r_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);


  if ((port_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY)) < 0) {
    RCLCPP_FATAL(rclcpp::get_logger("HtbSystem"), "Cannot open serial port to hoverboard");
    exit(-1);
  }
  RCLCPP_INFO(rclcpp::get_logger("HtbSystem"), "Opened serial port to hoverboard");


  for (const hardware_interface::ComponentInfo& joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(rclcpp::get_logger("HtbSystem"), "Joint '%s' has %zu command interfaces found. 1 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(rclcpp::get_logger("HtbSystem"), "Joint '%s' have %s command interfaces found. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(rclcpp::get_logger("HtbSystem"), "Joint '%s' has %zu state interface. 2 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(rclcpp::get_logger("HtbSystem"), "Joint '%s' have '%s' as first state interface. '%s' expected.",
                   joint.name.c_str(), joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(rclcpp::get_logger("HtbSystem"), "Joint '%s' have '%s' as second state interface. '%s' expected.",
                   joint.name.c_str(), joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }
  }


  node_ = std::make_shared<rclcpp::Node>("htb_system_node");
  executor_.add_node(node_);
  executor_thread_ =
      std::make_unique<std::thread>(std::bind(&rclcpp::executors::MultiThreadedExecutor::spin, &executor_));

  return CallbackReturn::SUCCESS;
}

CallbackReturn HtbSystem::on_configure(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(rclcpp::get_logger("HtbSystem"), "Configuring");
  // return CallbackReturn::SUCCESS;


  low_wrap_ = ENCODER_LOW_WRAP_FACTOR*(ENCODER_MAX - ENCODER_MIN) + ENCODER_MIN;
  high_wrap_ = ENCODER_HIGH_WRAP_FACTOR*(ENCODER_MAX - ENCODER_MIN) + ENCODER_MIN;
  last_wheelcountR_ = last_wheelcountL_ = 0;
  multR_ = multL_ = 0;

  struct termios options;
  tcgetattr(port_fd_, &options);
  options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;		//<Set baud rate
  options.c_iflag = IGNPAR;
  options.c_oflag = 0;
  options.c_lflag = 0;
  tcflush(port_fd_, TCIFLUSH);
  tcsetattr(port_fd_, TCSANOW, &options);

  last_read_ = node_->get_clock()->now();


  return hardware_interface::CallbackReturn::SUCCESS;
}

CallbackReturn HtbSystem::on_cleanup(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(rclcpp::get_logger("HtbSystem"), "Cleaning up");
  return CallbackReturn::SUCCESS;
}

CallbackReturn HtbSystem::on_activate(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(rclcpp::get_logger("HtbSystem"), "Activating");

  // TODO: Initialize pose and status

  RCLCPP_INFO(node_->get_logger(), "Activation successful");
  return CallbackReturn::SUCCESS;
}

CallbackReturn HtbSystem::on_deactivate(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(rclcpp::get_logger("HtbSystem"), "Deactivating");
  cleanup_node();
  return CallbackReturn::SUCCESS;
}

CallbackReturn HtbSystem::on_shutdown(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(rclcpp::get_logger("HtbSystem"), "Shutting down");
  cleanup_node();
  return CallbackReturn::SUCCESS;
}

CallbackReturn HtbSystem::on_error(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(rclcpp::get_logger("HtbSystem"), "Handling error");
  cleanup_node();
  return CallbackReturn::SUCCESS;
}

std::vector<StateInterface> HtbSystem::export_state_interfaces()
{
  std::vector<StateInterface> state_interfaces;


  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_POSITION, &wheel_l_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel));

  return state_interfaces;
}

std::vector<CommandInterface> HtbSystem::export_command_interfaces()
{
  std::vector<CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd_rad_s));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd_rad_s));


  return command_interfaces;
}

void HtbSystem::cleanup_node()
{
  // TODO: Clean and reset
  if (port_fd_ != -1) 
    close(port_fd_);
}

return_type HtbSystem::read(const rclcpp::Time& time, const rclcpp::Duration&)
{
  RCLCPP_DEBUG(rclcpp::get_logger("HtbSystem"), "Reading motors state");

  if (port_fd_ != -1) {
    unsigned char c;
    int i = 0, r = 0;

    while ((r = ::read(port_fd_, &c, 1)) > 0 && i++ < 1024)
      protocol_recv(c);

    if (i > 0)
      last_read_ = node_->get_clock()->now();

    if (r < 0 && errno != EAGAIN)
      RCLCPP_ERROR(rclcpp::get_logger("HtbSystem"), "Reading from serial %s failed: %d", port_.c_str(), r);
  }

  rclcpp::Time now = node_->get_clock()->now();

  if ((now - last_read_).seconds() > 1) {
    RCLCPP_FATAL(rclcpp::get_logger("HtbSystem"), "Timeout reading from serial %s failed", port_.c_str());
  } 

  return return_type::OK;
}

return_type HtbSystem::write(const rclcpp::Time&, const rclcpp::Duration&)
{
  RCLCPP_DEBUG(rclcpp::get_logger("HtbSystem"), "Writing to motors");

   if (port_fd_ == -1) {
    RCLCPP_ERROR(rclcpp::get_logger("HtbSystem"), "Attempt to write on closed serial");
    exit(-1);
  }
  // print wheel_l_.cmd and wheel_r_.cmd
  RCLCPP_DEBUG(rclcpp::get_logger("HtbSystem"), "Wheel L: %f, Wheel R: %f", wheel_l_.cmd_rad_s, wheel_r_.cmd_rad_s);

  // Calculate wheel velocity from command RAD/S to RPM
  wheel_l_.cmd_rpm = wheel_l_.cmd_rad_s * 60.0 / (2.0*M_PI);
  wheel_r_.cmd_rpm = wheel_r_.cmd_rad_s * 60.0 / (2.0*M_PI);

  // Calculate steering from difference of left and right
  // TODO: This is a very simple implementation, we should use a PID controller
  double speed = (wheel_l_.cmd_rpm + wheel_r_.cmd_rpm)/2.0;
  double steer = (wheel_l_.cmd_rpm - wheel_r_.cmd_rpm)*2.0;

  if (cfg_.inverted == 1){
    speed = -speed;
    // steer = -steer;
  }

  SerialCommand command;
  command.start = (uint16_t)START_FRAME;
  command.steer = (int16_t)steer;
  command.speed = (int16_t)speed;
  command.checksum = (uint16_t)(command.start ^ command.steer ^ command.speed);

  int rc = ::write(port_fd_, (const void*)&command, sizeof(command));
  if (rc < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("HtbSystem"), "Error writing to hoverboard serial port");
  }

  return return_type::OK;
}

void HtbSystem::protocol_recv (char byte) {
  start_frame_ = ((uint16_t)(byte) << 8) | (uint8_t)prev_byte_;

  // Read the start frame
  if (start_frame_ == START_FRAME) {
    p_ = (char*)&msg_;
    *p_++ = prev_byte_;
    *p_++ = byte;
    msg_len_ = 2;
  } else if (msg_len_ >= 2 && msg_len_ < (int) sizeof(SerialFeedback)) {
    // Otherwise just read the message content until the end
    *p_++ = byte;
    msg_len_++;
  }

  if (msg_len_ == sizeof(SerialFeedback)) {
    uint16_t checksum = (uint16_t)(
        msg_.start ^
        msg_.cmd1 ^
        msg_.cmd2 ^
        msg_.speedR_meas ^
        msg_.speedL_meas ^
        msg_.wheelR_cnt ^
        msg_.wheelL_cnt ^
        msg_.batVoltage ^
        msg_.boardTemp ^
        msg_.cmdLed);

    if (msg_.start == START_FRAME && msg_.checksum == checksum) {
      std_msgs::msg::Float64 f;

      f.data = (double)msg_.batVoltage/100.0;
      // voltage_pub_.publish(f);

      f.data = (double)msg_.boardTemp/10.0;
      // temp_pub.publish(f);

      // Convert RPM to RAD/S
      // joints[0].vel.data = direction_correction * (abs(msg.speedL_meas) * 0.10472);
      // joints[1].vel.data = direction_correction * (abs(msg.speedR_meas) * 0.10472);
      if (cfg_.inverted == 1){
        wheel_l_.vel = direction_correction_ * (-msg_.speedL_meas * 0.10472);
        wheel_r_.vel = direction_correction_ * (msg_.speedR_meas * 0.10472);
      } else {
        wheel_l_.vel = direction_correction_ * (msg_.speedL_meas * 0.10472);
        wheel_r_.vel = direction_correction_ * (-msg_.speedR_meas * 0.10472);
      }
      RCLCPP_INFO(rclcpp::get_logger("HtbSystem"), "Wheel velocity: %f, %f", wheel_l_.vel, wheel_r_.vel);

      // Process encoder values and update odometry
      on_encoder_update (msg_.wheelR_cnt, msg_.wheelL_cnt);
    } else {
      RCLCPP_WARN(rclcpp::get_logger("HtbSystem"), "Hoverboard checksum mismatch: %d vs %d", msg_.checksum, checksum);
    }
    msg_len_ = 0;
  }
  prev_byte_ = byte;
}

void HtbSystem::on_encoder_update (int16_t right, int16_t left) {
  double posL = 0.0, posR = 0.0;

  // Calculate wheel position in ticks, factoring in encoder wraps
  if (right < low_wrap_ && last_wheelcountR_ > high_wrap_)
    multR_++;
  else if (right > high_wrap_ && last_wheelcountR_ < low_wrap_)
    multR_--;
  posR = right + multR_*(ENCODER_MAX-ENCODER_MIN);
  last_wheelcountR_ = right;

  if (left < low_wrap_ && last_wheelcountL_ > high_wrap_)
    multL_++;
  else if (left > high_wrap_ && last_wheelcountL_ < low_wrap_)
    multL_--;
  posL = left + multL_*(ENCODER_MAX-ENCODER_MIN);
  last_wheelcountL_ = left;

  // When the board shuts down and restarts, wheel ticks are reset to zero so the robot can be suddently lost
  // This section accumulates ticks even if board shuts down and is restarted   
  static double lastPosL = 0.0, lastPosR = 0.0;
  static double lastPubPosL = 0.0, lastPubPosR = 0.0;
  static bool nodeStartFlag = true;

  //IF there has been a pause in receiving data AND the new number of ticks is close to zero, indicates a board restard
  //(the board seems to often report 1-3 ticks on startup instead of zero)
  //reset the last read ticks to the startup values
  RCLCPP_INFO(rclcpp::get_logger("HtbSystem"), "posL: %f, posR: %f", posL, posR);
  if((node_->get_clock()->now() - last_read_).seconds() > 0.2
      && abs(posL) < 5 && abs(posR) < 5){
    lastPosL = posL;
    lastPosR = posR;
  }
  double posLDiff = 0;
  double posRDiff = 0;

  //if node is just starting keep odom at zeros
  if(nodeStartFlag){
    nodeStartFlag = false;
  }else{
    posLDiff = posL - lastPosL;
    posRDiff = posR - lastPosR;
  }
  RCLCPP_INFO(rclcpp::get_logger("HtbSystem"), "posLDiff: %f, posRDiff: %f", posLDiff, posRDiff);

  lastPubPosL += posLDiff;
  lastPubPosR += posRDiff;
  lastPosL = posL;
  lastPosR = posR;

  // Convert position in accumulated ticks to position in radians
  // joints[0].pos.data = 2.0*M_PI * lastPubPosL/(double)TICKS_PER_ROTATION;
  // joints[1].pos.data = 2.0*M_PI * lastPubPosR/(double)TICKS_PER_ROTATION;
  wheel_l_.pos = 2.0*M_PI * lastPubPosL/(double)cfg_.enc_counts_per_rev;
  wheel_r_.pos = 2.0*M_PI * lastPubPosR/(double)cfg_.enc_counts_per_rev;
  RCLCPP_INFO(rclcpp::get_logger("HtbSystem"), "Wheel position: %f, %f", wheel_l_.pos, wheel_r_.pos);

  // pos_pub[0].publish(joints[0].pos);
  // pos_pub[1].publish(joints[1].pos);
}

}  // namespace htb_hardware_interfaces

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(htb_hardware_interfaces::HtbSystem, hardware_interface::SystemInterface)
