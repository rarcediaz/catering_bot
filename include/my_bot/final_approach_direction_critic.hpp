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

#ifndef MY_BOT__FINAL_APPROACH_DIRECTION_CRITIC_HPP_
#define MY_BOT__FINAL_APPROACH_DIRECTION_CRITIC_HPP_

#include <cstddef>

#include "nav2_mppi_controller/critic_function.hpp"

namespace my_bot
{

enum class TravelDirection : int
{
  REVERSE = -1,
  UNKNOWN = 0,
  FORWARD = 1,
};

struct DirectionLatchUpdate
{
  TravelDirection direction{TravelDirection::UNKNOWN};
  bool goal_changed{false};
  bool latch_acquired{false};
  bool forced_switch{false};
};

class FinalApproachDirectionLatch
{
public:
  FinalApproachDirectionLatch(
    double speed_threshold, double goal_change_tolerance,
    std::size_t opposite_confirmation_cycles);

  DirectionLatchUpdate update(
    bool inside_final_approach, double linear_speed, double goal_x, double goal_y);

  TravelDirection direction() const;

private:
  TravelDirection direction_from_speed(double linear_speed) const;
  bool goal_has_changed(double goal_x, double goal_y) const;
  void reset_for_goal(double goal_x, double goal_y);

  double speed_threshold_;
  double goal_change_tolerance_;
  std::size_t opposite_confirmation_cycles_;
  bool has_goal_{false};
  double goal_x_{0.0};
  double goal_y_{0.0};
  TravelDirection last_motion_direction_{TravelDirection::UNKNOWN};
  TravelDirection latched_direction_{TravelDirection::UNKNOWN};
  std::size_t opposite_observation_count_{0};
};

}  // namespace my_bot

namespace mppi::critics
{

class FinalApproachDirectionCritic : public CriticFunction
{
public:
  void initialize() override;
  void score(CriticData & data) override;

private:
  unsigned int power_{1};
  float weight_{20.0};
  float approach_distance_{1.4};
  float speed_threshold_{0.08};
  float goal_change_tolerance_{0.5};
  int opposite_confirmation_cycles_{5};
  my_bot::FinalApproachDirectionLatch latch_{0.08, 0.5, 5};
};

}  // namespace mppi::critics

#endif  // MY_BOT__FINAL_APPROACH_DIRECTION_CRITIC_HPP_
