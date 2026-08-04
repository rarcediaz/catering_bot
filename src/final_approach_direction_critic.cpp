// Copyright 2026 IntelliTrolley contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "my_bot/final_approach_direction_critic.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>

#include "nav2_mppi_controller/tools/utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "xtensor/xmath.hpp"
#include "xtensor/xoperation.hpp"
#include "xtensor/xreducer.hpp"

namespace my_bot
{

FinalApproachDirectionLatch::FinalApproachDirectionLatch(
  double speed_threshold, double goal_change_tolerance,
  std::size_t opposite_confirmation_cycles)
: speed_threshold_(std::max(0.0, speed_threshold)),
  goal_change_tolerance_(std::max(0.0, goal_change_tolerance)),
  opposite_confirmation_cycles_(std::max<std::size_t>(1, opposite_confirmation_cycles))
{
}

TravelDirection FinalApproachDirectionLatch::direction_from_speed(double linear_speed) const
{
  if (linear_speed >= speed_threshold_) {
    return TravelDirection::FORWARD;
  }
  if (linear_speed <= -speed_threshold_) {
    return TravelDirection::REVERSE;
  }
  return TravelDirection::UNKNOWN;
}

bool FinalApproachDirectionLatch::goal_has_changed(double goal_x, double goal_y) const
{
  if (!has_goal_) {
    return true;
  }
  return std::hypot(goal_x - goal_x_, goal_y - goal_y_) > goal_change_tolerance_;
}

void FinalApproachDirectionLatch::reset_for_goal(double goal_x, double goal_y)
{
  has_goal_ = true;
  goal_x_ = goal_x;
  goal_y_ = goal_y;
  last_motion_direction_ = TravelDirection::UNKNOWN;
  latched_direction_ = TravelDirection::UNKNOWN;
  opposite_observation_count_ = 0;
}

DirectionLatchUpdate FinalApproachDirectionLatch::update(
  bool inside_final_approach, double linear_speed, double goal_x, double goal_y)
{
  DirectionLatchUpdate result;
  if (goal_has_changed(goal_x, goal_y)) {
    reset_for_goal(goal_x, goal_y);
    result.goal_changed = true;
  }

  const auto observed_direction = direction_from_speed(linear_speed);
  if (!inside_final_approach) {
    latched_direction_ = TravelDirection::UNKNOWN;
    opposite_observation_count_ = 0;
    if (observed_direction != TravelDirection::UNKNOWN) {
      last_motion_direction_ = observed_direction;
    }
    result.direction = latched_direction_;
    return result;
  }

  const bool observed_opposite_direction =
    observed_direction != TravelDirection::UNKNOWN &&
    observed_direction != latched_direction_;
  if (latched_direction_ == TravelDirection::UNKNOWN) {
    const auto direction_to_latch = observed_direction != TravelDirection::UNKNOWN ?
      observed_direction : last_motion_direction_;
    if (direction_to_latch != TravelDirection::UNKNOWN) {
      latched_direction_ = direction_to_latch;
      last_motion_direction_ = direction_to_latch;
      result.latch_acquired = true;
    }
  } else if (observed_opposite_direction) {
    ++opposite_observation_count_;
    if (opposite_observation_count_ >= opposite_confirmation_cycles_) {
      latched_direction_ = observed_direction;
      last_motion_direction_ = observed_direction;
      opposite_observation_count_ = 0;
      result.forced_switch = true;
    }
  } else {
    opposite_observation_count_ = 0;
    if (observed_direction == latched_direction_) {
      last_motion_direction_ = observed_direction;
    }
  }

  result.direction = latched_direction_;
  return result;
}

TravelDirection FinalApproachDirectionLatch::direction() const
{
  return latched_direction_;
}

}  // namespace my_bot

namespace mppi::critics
{

void FinalApproachDirectionCritic::initialize()
{
  auto get_param = parameters_handler_->getParamGetter(name_);
  get_param(power_, "cost_power", 1);
  get_param(weight_, "cost_weight", 20.0f);
  get_param(approach_distance_, "approach_distance", 1.4f);
  get_param(speed_threshold_, "speed_threshold", 0.08f);
  get_param(goal_change_tolerance_, "goal_change_tolerance", 0.5f);
  get_param(opposite_confirmation_cycles_, "opposite_confirmation_cycles", 5);

  approach_distance_ = std::max(0.0f, approach_distance_);
  speed_threshold_ = std::max(0.0f, speed_threshold_);
  goal_change_tolerance_ = std::max(0.0f, goal_change_tolerance_);
  opposite_confirmation_cycles_ = std::max(1, opposite_confirmation_cycles_);
  latch_ = my_bot::FinalApproachDirectionLatch(
    speed_threshold_, goal_change_tolerance_,
    static_cast<std::size_t>(opposite_confirmation_cycles_));

  RCLCPP_INFO(
    logger_,
    "FinalApproachDirectionCritic active inside %.2f m with weight %.1f and %d-cycle "
    "opposite-direction confirmation.",
    approach_distance_, weight_, opposite_confirmation_cycles_);
}

void FinalApproachDirectionCritic::score(CriticData & data)
{
  using xt::evaluation_strategy::immediate;
  if (!enabled_ || data.path.x.size() == 0) {
    return;
  }

  const auto goal_index = data.path.x.size() - 1;
  const double goal_x = data.path.x(goal_index);
  const double goal_y = data.path.y(goal_index);
  const double dx = data.state.pose.pose.position.x - goal_x;
  const double dy = data.state.pose.pose.position.y - goal_y;
  const double goal_distance = std::hypot(dx, dy);
  const auto update = latch_.update(
    goal_distance <= approach_distance_, data.state.speed.linear.x, goal_x, goal_y);

  if (update.latch_acquired) {
    const char * direction = update.direction == my_bot::TravelDirection::FORWARD ?
      "forward" : "reverse";
    RCLCPP_INFO(
      logger_, "Final approach direction latched %s at %.2f m from goal.",
      direction, goal_distance);
  } else if (update.forced_switch) {
    const char * direction = update.direction == my_bot::TravelDirection::FORWARD ?
      "forward" : "reverse";
    RCLCPP_WARN(
      logger_,
      "Final approach direction changed to %s after %d consecutive controller cycles; "
      "the original direction could not be maintained.",
      direction, opposite_confirmation_cycles_);
  }

  const auto add_opposite_motion_cost = [&](const auto & opposite_motion) {
      data.costs += xt::pow(
        xt::sum(opposite_motion * data.model_dt, {1}, immediate) * weight_, power_);
    };

  if (update.direction == my_bot::TravelDirection::FORWARD) {
    add_opposite_motion_cost(xt::maximum(-data.state.vx, 0.0f));
  } else if (update.direction == my_bot::TravelDirection::REVERSE) {
    add_opposite_motion_cost(xt::maximum(data.state.vx, 0.0f));
  }
}

}  // namespace mppi::critics

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::FinalApproachDirectionCritic,
  mppi::critics::CriticFunction)
