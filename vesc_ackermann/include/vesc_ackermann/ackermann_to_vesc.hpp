#ifndef VESC_ACKERMANN__ACKERMANN_TO_VESC_HPP_
#define VESC_ACKERMANN__ACKERMANN_TO_VESC_HPP_

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

namespace vesc_ackermann
{

using ackermann_msgs::msg::AckermannDriveStamped;
using std_msgs::msg::Float64;

class AckermannToVesc : public rclcpp::Node
{
public:
  explicit AckermannToVesc(const rclcpp::NodeOptions & options);

private:
  // ROS parameters
  // conversion gain and offset
  double speed_to_erpm_gain_, speed_to_erpm_offset_;
  double steering_to_servo_gain_, steering_to_servo_offset_;

  // ramping and filtering
  double erpm_ramp_rate_;
  double erpm_lpf_alpha_;
  double current_erpm_;
  double target_erpm_;
  double filtered_erpm_;   // add this

  rclcpp::TimerBase::SharedPtr ramp_timer_;
  void ackermannCmdCallback(const AckermannDriveStamped::SharedPtr cmd);
  // ramp timer - steps current_erpm_ toward target_erpm_ at fixed rate
  void rampCallback();
  /** @todo consider also providing an interpolated look-up table conversion */

  // ROS services
  rclcpp::Publisher<Float64>::SharedPtr erpm_pub_;
  rclcpp::Publisher<Float64>::SharedPtr servo_pub_;
  rclcpp::Subscription<AckermannDriveStamped>::SharedPtr ackermann_sub_;

};

}  // namespace vesc_ackermann

#endif  // VESC_ACKERMANN__ACKERMANN_TO_VESC_HPP_