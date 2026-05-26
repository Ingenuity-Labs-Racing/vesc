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
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <std_msgs/msg/float64.hpp>

#include <cmath>
#include <sstream>
#include <string>
#include <vector>

namespace vesc_ackermann
{

using ackermann_msgs::msg::AckermannDriveStamped;
using std::placeholders::_1;
using std_msgs::msg::Float64;

AckermannToVesc::AckermannToVesc(const rclcpp::NodeOptions & options)
: Node("ackermann_to_vesc_node", options),
  current_erpm_(0.0),
  target_erpm_(0.0),
  filtered_erpm_(0.0)
{
  // ── Static conversion parameters ──────────────────────────────────────
  declare_parameter("speed_to_erpm_gain", 1.0);
  declare_parameter("speed_to_erpm_offset", 0.0);
  declare_parameter("steering_angle_to_servo_gain", 1.0);
  declare_parameter("steering_angle_to_servo_offset", 0.0);

  // ── Tunable parameters ────────────────────────────────────────────────
  //
  // erpm_ramp_rate  [ERPM/s]
  //   Controls how quickly the VESC is allowed to accelerate / decelerate.
  //   Ramp timer fires at 50 Hz (every 20 ms), so:
  //     max ERPM step per tick = erpm_ramp_rate * 0.02
  //
  //   Typical values:
  //     150 000  – aggressive (~0.2 s to full speed)
  //      55 000  – balanced   (~0.55 s)           ← default
  //      30 000  – gentle     (~1.0 s)
  //
  // erpm_lpf_alpha  [0.0–1.0]  (one-pole IIR coefficient applied to raw ERPM)
  //   Smooths noisy / jittery speed commands before they reach the ramp.
  //     0.0  – output never updates (frozen at 0)
  //     0.01 – very heavy smoothing, slow to respond  ← default
  //     0.1  – moderate smoothing
  //     1.0  – no filtering, raw command passed straight through
  //
  // Both parameters can be changed at runtime without restarting the node:
  //   ros2 param set /ackermann_to_vesc_node erpm_ramp_rate  80000.0
  //   ros2 param set /ackermann_to_vesc_node erpm_lpf_alpha  0.05

  rcl_interfaces::msg::ParameterDescriptor ramp_desc;
  ramp_desc.description =
    "Maximum ERPM change per second. Tunable at runtime via ros2 param set.";
  ramp_desc.read_only = false;
  declare_parameter("erpm_ramp_rate", 55000.0, ramp_desc);

  rcl_interfaces::msg::ParameterDescriptor lpf_desc;
  lpf_desc.description =
    "One-pole IIR smoothing factor [0.0–1.0]. "
    "Low values = heavy filtering, high values = raw command. "
    "Tunable at runtime via ros2 param set.";
  lpf_desc.read_only = false;
  declare_parameter("erpm_lpf_alpha", 0.01, lpf_desc);

  // ── Read all parameters ────────────────────────────────────────────────
  get_parameter("speed_to_erpm_gain",          speed_to_erpm_gain_);
  get_parameter("speed_to_erpm_offset",        speed_to_erpm_offset_);
  get_parameter("steering_angle_to_servo_gain", steering_to_servo_gain_);
  get_parameter("steering_angle_to_servo_offset", steering_to_servo_offset_);
  get_parameter("erpm_ramp_rate",  erpm_ramp_rate_);
  get_parameter("erpm_lpf_alpha",  erpm_lpf_alpha_);

  // ── Register runtime parameter callback ───────────────────────────────
  // Only erpm_ramp_rate and erpm_lpf_alpha are accepted; all other
  // set-parameter calls are forwarded as successful without side-effects.
  param_cb_handle_ = add_on_set_parameters_callback(
    std::bind(&AckermannToVesc::onSetParameters, this, _1));

  // ── Publishers ────────────────────────────────────────────────────────
  erpm_pub_  = create_publisher<Float64>("commands/motor/speed",    10);
  servo_pub_ = create_publisher<Float64>("commands/servo/position", 10);

  // ── Subscriber ────────────────────────────────────────────────────────
  ackermann_sub_ = create_subscription<AckermannDriveStamped>(
    "ackermann_cmd", 10,
    std::bind(&AckermannToVesc::ackermannCmdCallback, this, _1));

  // ── Ramp timer (50 Hz) ────────────────────────────────────────────────
  ramp_timer_ = create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&AckermannToVesc::rampCallback, this));

  RCLCPP_INFO(
    get_logger(),
    "AckermannToVesc ready — erpm_ramp_rate=%.0f ERPM/s, erpm_lpf_alpha=%.3f",
    erpm_ramp_rate_, erpm_lpf_alpha_);
}

// ── Parameter callback ──────────────────────────────────────────────────────

rcl_interfaces::msg::SetParametersResult
AckermannToVesc::onSetParameters(const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & param : params) {
    const std::string & name = param.get_name();

    if (name == "erpm_ramp_rate") {
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        result.successful = false;
        result.reason = "erpm_ramp_rate must be a double";
        return result;
      }
      double new_rate = param.as_double();
      if (new_rate <= 0.0) {
        result.successful = false;
        result.reason = "erpm_ramp_rate must be > 0";
        return result;
      }
      erpm_ramp_rate_ = new_rate;
      RCLCPP_INFO(get_logger(), "erpm_ramp_rate updated → %.0f ERPM/s", erpm_ramp_rate_);

    } else if (name == "erpm_lpf_alpha") {
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        result.successful = false;
        result.reason = "erpm_lpf_alpha must be a double";
        return result;
      }
      double new_alpha = param.as_double();
      if (new_alpha < 0.0 || new_alpha > 1.0) {
        result.successful = false;
        result.reason = "erpm_lpf_alpha must be in [0.0, 1.0]";
        return result;
      }
      erpm_lpf_alpha_ = new_alpha;
      RCLCPP_INFO(get_logger(), "erpm_lpf_alpha updated → %.4f", erpm_lpf_alpha_);
    }
    // All other parameters are passed through silently.
  }

  return result;
}

// ── Ackermann callback ──────────────────────────────────────────────────────

void AckermannToVesc::ackermannCmdCallback(const AckermannDriveStamped::SharedPtr cmd)
{
  // Convert incoming speed to raw ERPM.
  double raw_erpm = speed_to_erpm_gain_ * cmd->drive.speed + speed_to_erpm_offset_;

  // One-pole IIR low-pass filter:
  //   filtered = alpha * raw + (1 - alpha) * filtered_prev
  //
  // alpha=0.01 → ~99 % of previous value retained per sample → heavy smoothing.
  // alpha=1.0  → no filtering (filtered == raw every tick).
  filtered_erpm_ = erpm_lpf_alpha_ * raw_erpm + (1.0 - erpm_lpf_alpha_) * filtered_erpm_;
  target_erpm_   = filtered_erpm_;

  // Steering has no need for smoothing — publish immediately.
  Float64 servo_msg;
  servo_msg.data =
    steering_to_servo_gain_ * cmd->drive.steering_angle + steering_to_servo_offset_;

  if (rclcpp::ok()) {
    servo_pub_->publish(servo_msg);
  }
}

// ── Ramp timer callback (50 Hz) ─────────────────────────────────────────────

void AckermannToVesc::rampCallback()
{
  // Maximum ERPM change allowed in this 20 ms tick.
  const double max_step = erpm_ramp_rate_ * 0.02;
  const double error    = target_erpm_ - current_erpm_;

  if (std::abs(error) <= max_step) {
    // Close enough — snap exactly to avoid floating-point drift.
    current_erpm_ = target_erpm_;
  } else {
    // Step toward target, preserving direction.
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