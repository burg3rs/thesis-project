#pragma once
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace ackermann_chassis {

class ArduinoSerialHardware : public hardware_interface::SystemInterface {
public:
  // ros2_control hooks
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  // params
  std::string port_{"/dev/ttyACM0"};
  int baud_{115200};
  double wheel_radius_{0.05};
  double max_mps_{1.5};
  double steer_limit_rad_{0.837758};

  // last transmitter commands
  double last_sent_steer_deg_{9999.0};
  int last_sent_pwmL_{9999};
  int last_sent_pwmR_{9999};

  // resend/heartbeat logic
  rclcpp::Time last_write_time_{0,0,RCL_ROS_TIME};

  double resend_period_{0.25};
  double epsilon_{1e-4};
  
// commands (FL positions; RL, RR velocities)
  double cmd_pos_fl_{0.0};
  double cmd_vel_rl_{0.0}, cmd_vel_rr_{0.0};

  // states
  double state_pos_fl_{0.0}, state_pos_fr_{0.0};
  double state_vel_rl_{0.0}, state_vel_rr_{0.0};
  double state_pos_rl_{0.0}, state_pos_rr_{0.0};

  // POSIX serial
  int fd_{-1};
  std::string rx_buf_;

  // helpers
  static inline double rad2deg(double r) { return r * 180.0 / M_PI; }
  int clamp_pwm(double mps) const;

  bool openSerial();
  void closeSerial();
  void writeLine(const std::string &s);
  void pollSerial();
  void parseFeedbackLine(const std::string &line);
};

} // namespace ackermann_chassis
