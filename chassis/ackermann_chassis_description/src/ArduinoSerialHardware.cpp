#include "ackermann_chassis/ArduinoSerialHardware.hpp"
#include <pluginlib/class_list_macros.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cmath>

#include <cstring>
#include <sstream>
#include <chrono>
#include <thread>

namespace ackermann_chassis {

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

static speed_t baud_to_termios(int baud) {
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: default: return B115200;
  }
}

// Function to to read configuration from the URDF and store parameter values
CallbackReturn ArduinoSerialHardware::on_init(const hardware_interface::HardwareInfo & info) {
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) return CallbackReturn::ERROR;
  const auto &p = info.hardware_parameters;
  if (p.count("port")) port_ = p.at("port");
  if (p.count("baud")) baud_ = std::stoi(p.at("baud"));
  if (p.count("wheel_radius")) wheel_radius_ = std::stod(p.at("wheel_radius"));
  if (p.count("max_mps")) max_mps_ = std::stod(p.at("max_mps"));
  if (p.count("steer_limit_rad")) steer_limit_rad_ = std::stod(p.at("steer_limit_rad"));
  return CallbackReturn::SUCCESS;
}

// Function to tell ros2_control which states the hardware is providing
std::vector<hardware_interface::StateInterface>
ArduinoSerialHardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> si;

  // steering (position only)
  si.emplace_back(info_.joints[0].name, hardware_interface::HW_IF_POSITION, &state_pos_fl_);
  si.emplace_back(info_.joints[1].name, hardware_interface::HW_IF_POSITION, &state_pos_fr_);

  // rear left (position + velocity as will be used for odometry for robot motion) 
  si.emplace_back(info_.joints[2].name, hardware_interface::HW_IF_POSITION, &state_pos_rl_); 
  si.emplace_back(info_.joints[2].name, hardware_interface::HW_IF_VELOCITY, &state_vel_rl_);

  // rear right (position + velocity)
  si.emplace_back(info_.joints[3].name, hardware_interface::HW_IF_POSITION, &state_pos_rr_);
  si.emplace_back(info_.joints[3].name, hardware_interface::HW_IF_VELOCITY, &state_vel_rr_);

  return si;
}

// Function for ros2_control to know what joint commands the hardware can receive
std::vector<hardware_interface::CommandInterface>
ArduinoSerialHardware::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> ci;
  ci.reserve(3);
  ci.emplace_back(info_.joints[0].name, hardware_interface::HW_IF_POSITION, &cmd_pos_fl_); //front_left
  ci.emplace_back(info_.joints[2].name, hardware_interface::HW_IF_VELOCITY, &cmd_vel_rl_); //rear_left
  ci.emplace_back(info_.joints[3].name, hardware_interface::HW_IF_VELOCITY, &cmd_vel_rr_); //rear_right
  return ci;
}

// Function for opening and configuring the serial connection to the Arduino
bool ArduinoSerialHardware::openSerial() {
  fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("ArduinoSerialHardware"), "Failed to open %s", port_.c_str());
    return false;
  }
  termios tio{};
  if (tcgetattr(fd_, &tio) != 0) { ::close(fd_); fd_ = -1; return false; }
  cfmakeraw(&tio);
  speed_t sp = baud_to_termios(baud_);
  cfsetispeed(&tio, sp);
  cfsetospeed(&tio, sp);
  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cflag &= ~CRTSCTS;
  tio.c_cc[VMIN]  = 0;
  tio.c_cc[VTIME] = 0;
  if (tcsetattr(fd_, TCSANOW, &tio) != 0) { ::close(fd_); fd_ = -1; return false; }
  return true;
}

// Function to safely close the serial file descriptor if it is open
void ArduinoSerialHardware::closeSerial() {
  if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

hardware_interface::CallbackReturn ArduinoSerialHardware::on_configure(const rclcpp_lifecycle::State &) {
  if (!openSerial()) return CallbackReturn::ERROR;
 
  std::this_thread::sleep_for(std::chrono::seconds(2));
  
  last_sent_steer_deg_ = 9999.0;
  last_sent_pwmL_ = 9999;
  last_sent_pwmR_ = 9999;
  last_write_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  RCLCPP_INFO(rclcpp::get_logger("ArduinoSerialHardware"), "Opened %s @ %d", port_.c_str(), baud_);
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArduinoSerialHardware::on_cleanup(const rclcpp_lifecycle::State &) {
  closeSerial();
  return CallbackReturn::SUCCESS;
}

// Function that converts a value of meters per second into a PWM the motor can use
int ArduinoSerialHardware::clamp_pwm(double mps) const {
  if (max_mps_ <= 0.0) return 0;
  double pwm = (mps / max_mps_) * 100.0;
  if (pwm > 100.0) pwm = 100.0;
  if (pwm < -100.0) pwm = -100.0;
  return static_cast<int>(std::lround(pwm));
}

void ArduinoSerialHardware::writeLine(const std::string &s) {
  if (fd_ < 0) return;

  //RCLCPP_INFO(
  //  rclcpp::get_logger("ArduinoSerialHardware"),
  //  "SERIAL TX: %s",
  //  s.c_str());

  (void)::write(fd_, s.c_str(), s.size());

  ::tcdrain(fd_);
}

void ArduinoSerialHardware::parseFeedbackLine(const std::string &line) {
  // FB:VL_MPS:<L>,VR_MPS:<R>,...
  if (line.rfind("FB:", 0) != 0) return;

  auto find_val = [&](const char *key, double &out)->bool {
    auto k = line.find(key);
    if (k == std::string::npos) return false;
    k += std::strlen(key);
    size_t e = line.find_first_of(", \r\n", k);
    std::string s = line.substr(k, (e == std::string::npos) ? std::string::npos : e - k);
    try { out = std::stod(s); } catch (...) { return false; }
    return true;
  };

  double vl_mps = 0.0, vr_mps = 0.0;
  if (find_val("VL_MPS:", vl_mps)) state_vel_rl_ = wheel_radius_ > 0.0 ? (vl_mps / wheel_radius_) : 0.0;
  if (find_val("VR_MPS:", vr_mps)) state_vel_rr_ = wheel_radius_ > 0.0 ? (vr_mps / wheel_radius_) : 0.0;

  // mirror steering (no feedback channel)
  state_pos_fl_ = cmd_pos_fl_;
  state_pos_fr_ = state_pos_fl_;
}

void ArduinoSerialHardware::pollSerial() {
  if (fd_ < 0) return;
  char buf[256];
  ssize_t n = ::read(fd_, buf, sizeof(buf));
  if (n <= 0) return;
  rx_buf_.append(buf, buf + n);
  size_t pos;
  while ((pos = rx_buf_.find('\n')) != std::string::npos) {
    std::string line = rx_buf_.substr(0, pos);
    rx_buf_.erase(0, pos + 1);
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (!line.empty()) parseFeedbackLine(line);
  }
}

hardware_interface::return_type ArduinoSerialHardware::read(
    const rclcpp::Time & time, const rclcpp::Duration & period) {
  pollSerial();

  // steering states follow commanded position
  state_pos_fl_ = cmd_pos_fl_;
  state_pos_fr_ = state_pos_fl_;

  // integrate rear wheel angular position from angular velocity
  const double dt = period.seconds();
  state_pos_rl_ += state_vel_rl_ * dt;
  state_pos_rr_ += state_vel_rr_ * dt;

  return return_type::OK;
}

hardware_interface::return_type ArduinoSerialHardware::write(const rclcpp::Time & current_time, const rclcpp::Duration & period) {
  // Single front steering joint → one servo command (deg relative)
  double steer_rad = cmd_pos_fl_;
  if (steer_rad >  steer_limit_rad_) steer_rad =  steer_limit_rad_;
  if (steer_rad < -steer_limit_rad_) steer_rad = -steer_limit_rad_;
  
  double steer_deg_rel = rad2deg(steer_rad);

  // Rear wheels: rad/s → m/s → PWM (−100..100)
  double vL_mps = cmd_vel_rl_ * wheel_radius_;
  double vR_mps = cmd_vel_rr_ * wheel_radius_;
  int pwmL = clamp_pwm(vL_mps);
  int pwmR = clamp_pwm(vR_mps);

  auto now_time = current_time;

  bool changed =
  std::abs(steer_deg_rel - last_sent_steer_deg_) > 1.0 ||
  pwmL != last_sent_pwmL_ ||
  pwmR != last_sent_pwmR_;

  bool heartbeat =
  (now_time - last_write_time_).seconds() > resend_period_;

  if (changed || heartbeat) {
  {
    std::ostringstream os;
    os << "STEER:" << steer_deg_rel << "\n";
    writeLine(os.str());
  }

  {
    std::ostringstream s;
    s << "VEL_L:" << pwmL << ",VEL_R:" << pwmR << "\n";
    writeLine(s.str());
  }

 if (changed) {
   RCLCPP_INFO(
     rclcpp::get_logger("ArduinoSerialHardware"),
     "RAW CMD fl=%.3f rl=%.3f rr=%.3f | steer_deg=%.2f pwmL=%d pwmR=%d",
     cmd_pos_fl_, cmd_vel_rl_, cmd_vel_rr_,
     steer_deg_rel, pwmL, pwmR);
  }
  
  last_sent_steer_deg_ = steer_deg_rel;
  last_sent_pwmL_ = pwmL;
  last_sent_pwmR_ = pwmR;
  last_write_time_ = now_time;
}
  return return_type::OK;
}

} // namespace ackermann_chassis

PLUGINLIB_EXPORT_CLASS(ackermann_chassis::ArduinoSerialHardware,
                       hardware_interface::SystemInterface)
