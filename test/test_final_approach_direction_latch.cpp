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

#include "gtest/gtest.h"

using my_bot::FinalApproachDirectionLatch;
using my_bot::TravelDirection;

TEST(FinalApproachDirectionLatch, PreservesForwardEntryDirection)
{
  FinalApproachDirectionLatch latch(0.08, 0.5, 5);
  latch.update(false, 0.35, 10.0, 20.0);
  const auto update = latch.update(true, 0.02, 10.0, 20.0);

  EXPECT_TRUE(update.latch_acquired);
  EXPECT_EQ(update.direction, TravelDirection::FORWARD);
}

TEST(FinalApproachDirectionLatch, PreservesReverseEntryDirection)
{
  FinalApproachDirectionLatch latch(0.08, 0.5, 5);
  latch.update(false, -0.30, 10.0, 20.0);
  const auto update = latch.update(true, -0.04, 10.0, 20.0);

  EXPECT_TRUE(update.latch_acquired);
  EXPECT_EQ(update.direction, TravelDirection::REVERSE);
}

TEST(FinalApproachDirectionLatch, IgnoresBriefOppositeMotion)
{
  FinalApproachDirectionLatch latch(0.08, 0.5, 5);
  latch.update(false, 0.30, 10.0, 20.0);
  latch.update(true, 0.30, 10.0, 20.0);

  for (int cycle = 0; cycle < 4; ++cycle) {
    const auto update = latch.update(true, -0.20, 10.0, 20.0);
    EXPECT_FALSE(update.forced_switch);
    EXPECT_EQ(update.direction, TravelDirection::FORWARD);
  }
}

TEST(FinalApproachDirectionLatch, AdoptsAConfirmedForcedDirectionChange)
{
  FinalApproachDirectionLatch latch(0.08, 0.5, 5);
  latch.update(false, 0.30, 10.0, 20.0);
  latch.update(true, 0.30, 10.0, 20.0);

  for (int cycle = 0; cycle < 4; ++cycle) {
    latch.update(true, -0.20, 10.0, 20.0);
  }
  const auto update = latch.update(true, -0.20, 10.0, 20.0);

  EXPECT_TRUE(update.forced_switch);
  EXPECT_EQ(update.direction, TravelDirection::REVERSE);
}

TEST(FinalApproachDirectionLatch, ChangedGoalResetsThePreviousMissionDirection)
{
  FinalApproachDirectionLatch latch(0.08, 0.5, 5);
  latch.update(false, 0.30, 10.0, 20.0);
  latch.update(true, 0.30, 10.0, 20.0);
  const auto update = latch.update(true, -0.30, 12.0, 20.0);

  EXPECT_TRUE(update.goal_changed);
  EXPECT_TRUE(update.latch_acquired);
  EXPECT_EQ(update.direction, TravelDirection::REVERSE);
}

TEST(FinalApproachDirectionLatch, LeavingFinalApproachAllowsANewEntryDirection)
{
  FinalApproachDirectionLatch latch(0.08, 0.5, 5);
  latch.update(false, 0.30, 10.0, 20.0);
  latch.update(true, 0.30, 10.0, 20.0);
  latch.update(false, -0.30, 10.0, 20.0);
  const auto update = latch.update(true, -0.30, 10.0, 20.0);

  EXPECT_TRUE(update.latch_acquired);
  EXPECT_EQ(update.direction, TravelDirection::REVERSE);
}
