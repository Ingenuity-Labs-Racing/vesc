// // Copyright 2020 F1TENTH Foundation
// //
// // Redistribution and use in source and binary forms, with or without
// // modification, are permitted provided that the following conditions are met:
// //
// //   * Redistributions of source code must retain the above copyright
// //     notice, this list of conditions and the following disclaimer.
// //
// //   * Redistributions in binary form must reproduce the above copyright
// //     notice, this list of conditions and the following disclaimer in the
// //     documentation and/or other materials provided with the distribution.
// //
// //   * Neither the name of the {copyright_holder} nor the names of its
// //     contributors may be used to endorse or promote products derived from
// //     this software without specific prior written permission.
// //
// // THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// // AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// // IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// // ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// // LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// // CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// // SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// // INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// // CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// // ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// // POSSIBILITY OF SUCH DAMAGE.
// // -*- mode:c++; fill-column: 100; -*-

// #include "vesc_ackermann/ackermann_to_vesc.hpp"

// #include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
// #include <std_msgs/msg/float64.hpp>

// #include <cmath>
// #include <sstream>
// #include <string>

// namespace vesc_ackermann
// {

// using ackermann_msgs::msg::AckermannDriveStamped;
// using std::placeholders::_1;
// using std_msgs::msg::Float64;

// AckermannToVesc::AckermannToVesc(const rclcpp::NodeOptions & options)
// : Node("ackermann_to_vesc_node", options),
//   current_erpm_(0.0),
//   target_erpm_(0.0)
// {
//   // existing conversion parameters
//   declare_parameter("speed_to_erpm_gain", 1.0);
//   declare_parameter("speed_to_erpm_offset", 1.0);
//   declare_parameter("steering_angle_to_servo_gain", 1.0);
//   declare_parameter("steering_angle_to_servo_offset", 1.0);

//   // ramp rate parameter: max ERPM change per second
//   // 75000 ERPM/s = roughly 0.4s to reach full speed from standstill
//   // tune this value in your launch file to adjust aggressiveness:
//   //   150000 = aggressive (~0.2s)
//   //    75000 = recommended start (~0.4s)
//   //    50000 = conservative (~0.6s)
//   declare_parameter("erpm_ramp_rate", 75000.0);

//   get_parameter("speed_to_erpm_gain", speed_to_erpm_gain_);
//   get_parameter("speed_to_erpm_offset", speed_to_erpm_offset_);
//   get_parameter("steering_angle_to_servo_gain", steering_to_servo_gain_);
//   get_parameter("steering_angle_to_servo_offset", steering_to_servo_offset_);
//   get_parameter("erpm_ramp_rate", erpm_ramp_rate_);

//   // publishers
//   erpm_pub_ = create_publisher<Float64>("commands/motor/speed", 10);
//   servo_pub_ = create_publisher<Float64>("commands/servo/position", 10);

//   // subscribe to ackermann topic
//   ackermann_sub_ = create_subscription<AckermannDriveStamped>(
//     "ackermann_cmd", 10,
//     std::bind(&AckermannToVesc::ackermannCmdCallback, this, _1));

//   // ramp timer runs at 50Hz (every 20ms)
//   // each tick steps current_erpm_ toward target_erpm_ by at most:
//   //   max_step = erpm_ramp_rate * 0.02
//   //            = 75000 * 0.02 = 1500 ERPM per tick
//   ramp_timer_ = create_wall_timer(
//     std::chrono::milliseconds(20),
//     std::bind(&AckermannToVesc::rampCallback, this));
// }

// void AckermannToVesc::ackermannCmdCallback(const AckermannDriveStamped::SharedPtr cmd)
// {
//   // update target ERPM only - do NOT publish motor command directly
//   // the ramp timer is the only thing that publishes to the VESC
//   target_erpm_ = speed_to_erpm_gain_ * cmd->drive.speed + speed_to_erpm_offset_;

//   // steering does not need ramping, publish immediately as before
//   Float64 servo_msg;
//   servo_msg.data =
//     steering_to_servo_gain_ * cmd->drive.steering_angle + steering_to_servo_offset_;

//   if (rclcpp::ok()) {
//     servo_pub_->publish(servo_msg);
//   }
// }

// void AckermannToVesc::rampCallback()
// {
//   // maximum ERPM change allowed in this 20ms tick
//   double max_step = erpm_ramp_rate_ * 0.02;

//   double error = target_erpm_ - current_erpm_;

//   if (std::abs(error) <= max_step) {
//     // within one step of target, snap to it exactly
//     current_erpm_ = target_erpm_;
//   } else {
//     // step toward target, preserving sign direction
//     current_erpm_ += std::copysign(max_step, error);
//   }

//   Float64 erpm_msg;
//   erpm_msg.data = current_erpm_;

//   if (rclcpp::ok()) {
//     erpm_pub_->publish(erpm_msg);
//   }
// }

// }  // namespace vesc_ackermann

// #include "rclcpp_components/register_node_macro.hpp"  // NOLINT
// RCLCPP_COMPONENTS_REGISTER_NODE(vesc_ackermann::AckermannToVesc)

// Copyright 2020 F1TENTH Foundation
//
// (licence header unchanged)

#include "vesc_ackermann/ackermann_to_vesc.hpp"
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <cmath>
#include <numeric>
#include <deque>
#include <sstream>
#include <string>

namespace vesc_ackermann
{

using ackermann_msgs::msg::AckermannDriveStamped;
using std::placeholders::_1;
using std_msgs::msg::Float64;

AckermannToVesc::AckermannToVesc(const rclcpp::NodeOptions & options)
: Node("ackermann_to_vesc_node", options),
  current_erpm_(0.0),
  target_erpm_(0.0)
{
  // existing parameters
  declare_parameter("speed_to_erpm_gain", 1.0);
  declare_parameter("speed_to_erpm_offset", 1.0);
  declare_parameter("steering_angle_to_servo_gain", 1.0);
  declare_parameter("steering_angle_to_servo_offset", 1.0);

  // ramp rate: max ERPM change per second
  declare_parameter("erpm_ramp_rate", 55000.0);

  // moving average window size
  // 20 samples at typical 50Hz callback = 0.4s of averaging
  // larger window = smoother but more lag
  declare_parameter("erpm_ma_window", 20);

  get_parameter("speed_to_erpm_gain", speed_to_erpm_gain_);
  get_parameter("speed_to_erpm_offset", speed_to_erpm_offset_);
  get_parameter("steering_angle_to_servo_gain", steering_to_servo_gain_);
  get_parameter("steering_angle_to_servo_offset", steering_to_servo_offset_);
  get_parameter("erpm_ramp_rate", erpm_ramp_rate_);
  get_parameter("erpm_ma_window", erpm_ma_window_);

  // publishers
  erpm_pub_ = create_publisher<Float64>("commands/motor/speed", 10);
  servo_pub_ = create_publisher<Float64>("commands/servo/position", 10);

  // subscriber
  ackermann_sub_ = create_subscription<AckermannDriveStamped>(
    "ackermann_cmd", 10,
    std::bind(&AckermannToVesc::ackermannCmdCallback, this, _1));

  // ramp timer at 50Hz
  ramp_timer_ = create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&AckermannToVesc::rampCallback, this));
}

void AckermannToVesc::ackermannCmdCallback(const AckermannDriveStamped::SharedPtr cmd)
{
  // convert incoming speed to raw ERPM
  double raw_erpm = speed_to_erpm_gain_ * cmd->drive.speed + speed_to_erpm_offset_;

  // add to moving average window
  erpm_window_.push_back(raw_erpm);

  // drop oldest sample if window is full
  if (static_cast<int>(erpm_window_.size()) > erpm_ma_window_) {
    erpm_window_.pop_front();
  }

  // average all samples in window
  double sum = std::accumulate(erpm_window_.begin(), erpm_window_.end(), 0.0);
  target_erpm_ = sum / static_cast<double>(erpm_window_.size());

  // steering published immediately
  Float64 servo_msg;
  servo_msg.data =
    steering_to_servo_gain_ * cmd->drive.steering_angle + steering_to_servo_offset_;

  if (rclcpp::ok()) {
    servo_pub_->publish(servo_msg);
  }
}

void AckermannToVesc::rampCallback()
{
  double max_step = erpm_ramp_rate_ * 0.02;
  double error = target_erpm_ - current_erpm_;

  if (std::abs(error) <= max_step) {
    current_erpm_ = target_erpm_;
  } else {
    current_erpm_ += std::copysign(max_step, error);
  }

  Float64 erpm_msg;
  erpm_msg.data = current_erpm_;

  if (rclcpp::ok()) {
    erpm_pub_->publish(erpm_msg);
  }
}

}  // namespace vesc_ackermann

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(vesc_ackermann::AckermannToVesc)