#ifndef VESC_ACKERMANN__ACKERMANN_TO_VESC_HPP_
#define VESC_ACKERMANN__ACKERMANN_TO_VESC_HPP_

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <vector>

namespace vesc_ackermann
{

using ackermann_msgs::msg::AckermannDriveStamped;
using std_msgs::msg::Float64;

class AckermannToVesc : public rclcpp::Node
{
public:
  explicit AckermannToVesc(const rclcpp::NodeOptions & options);

private:
  // ── Conversion parameters (static) ──────────────────────────────────────
  double speed_to_erpm_gain_;
  double speed_to_erpm_offset_;
  double steering_to_servo_gain_;
  double steering_to_servo_offset_;

  // ── Tunable parameters (hot-swappable via ros2 param set) ───────────────
  double erpm_ramp_rate_;   ///< Max ERPM change per second [ERPM/s]
  double erpm_lpf_alpha_;   ///< Low-pass smoothing factor  [0.0–1.0]
                            ///<   0.0 = no update (frozen), 1.0 = no filter

  // ── Runtime state ───────────────────────────────────────────────────────
  double current_erpm_;     ///< Ramped ERPM actually sent to VESC
  double target_erpm_;      ///< LPF-smoothed ERPM that ramp tracks
  double filtered_erpm_;    ///< Accumulator for the one-pole IIR filter

  // ── ROS 2 interfaces ────────────────────────────────────────────────────
  rclcpp::Publisher<Float64>::SharedPtr erpm_pub_;
  rclcpp::Publisher<Float64>::SharedPtr servo_pub_;
  rclcpp::Subscription<AckermannDriveStamped>::SharedPtr ackermann_sub_;
  rclcpp::TimerBase::SharedPtr ramp_timer_;

  /// Handle kept alive so the callback is not unregistered on destruction.
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // ── Callbacks ───────────────────────────────────────────────────────────
  void ackermannCmdCallback(const AckermannDriveStamped::SharedPtr cmd);
  void rampCallback();

  /// Validates and applies runtime parameter changes.
  rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter> & params);
};

}  // namespace vesc_ackermann

#endif  // VESC_ACKERMANN__ACKERMANN_TO_VESC_HPP_