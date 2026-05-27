// Copyright 2020 F1TENTH Foundation
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//
//   * Neither the name of the {copyright_holder} nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
// -*- mode:c++; fill-column: 100; -*-

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
: Node("ackermann_to_vesc_node", options)
{
  // conversion parameters
  declare_parameter("speed_to_erpm_gain", 1.0);
  declare_parameter("speed_to_erpm_offset", 1.0);
  declare_parameter("steering_angle_to_servo_gain", 1.0);
  declare_parameter("steering_angle_to_servo_offset", 1.0);

  // steering moving average window size
  // at typical ackermann_cmd publish rate (~40-50Hz):
  //   5  samples = ~0.1s of smoothing  (recommended start)
  //   10 samples = ~0.2s of smoothing
  //   20 samples = ~0.4s of smoothing  (heavy, adds noticeable lag)
  declare_parameter("servo_ma_window", 5);

  get_parameter("speed_to_erpm_gain", speed_to_erpm_gain_);
  get_parameter("speed_to_erpm_offset", speed_to_erpm_offset_);
  get_parameter("steering_angle_to_servo_gain", steering_to_servo_gain_);
  get_parameter("steering_angle_to_servo_offset", steering_to_servo_offset_);
  get_parameter("servo_ma_window", servo_ma_window_);

  // publishers
  erpm_pub_ = create_publisher<Float64>("commands/motor/speed", 10);
  servo_pub_ = create_publisher<Float64>("commands/servo/position", 10);

  // subscriber
  ackermann_sub_ = create_subscription<AckermannDriveStamped>(
    "ackermann_cmd", 10,
    std::bind(&AckermannToVesc::ackermannCmdCallback, this, _1));
}

void AckermannToVesc::ackermannCmdCallback(const AckermannDriveStamped::SharedPtr cmd)
{
  // speed — unchanged, published directly
  Float64 erpm_msg;
  erpm_msg.data = speed_to_erpm_gain_ * cmd->drive.speed + speed_to_erpm_offset_;

  // steering — apply moving average before publishing
  double raw_servo = steering_to_servo_gain_ * cmd->drive.steering_angle +
                     steering_to_servo_offset_;

  servo_window_.push_back(raw_servo);
  if (static_cast<int>(servo_window_.size()) > servo_ma_window_) {
    servo_window_.pop_front();
  }
  double servo_sum = std::accumulate(servo_window_.begin(), servo_window_.end(), 0.0);
  double filtered_servo = servo_sum / static_cast<double>(servo_window_.size());

  Float64 servo_msg;
  servo_msg.data = filtered_servo;

  if (rclcpp::ok()) {
    erpm_pub_->publish(erpm_msg);
    servo_pub_->publish(servo_msg);
  }
}

}  // namespace vesc_ackermann

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(vesc_ackermann::AckermannToVesc)