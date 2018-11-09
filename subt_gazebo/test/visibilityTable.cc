/*
 * Copyright (C) 2018 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <gtest/gtest.h>
#include <ignition/common/Filesystem.hh>
#include "subt_gazebo/VisibilityTable.hh"
#include "test/test_config.h"

using namespace subt;

/////////////////////////////////////////////////
TEST(VisibilityTableTest, Visibility)
{
  std::string filePath = ignition::common::joinPaths(
    SUBT_GAZEBO_PROJECT_SOURCE_PATH, "worlds", "tunnel_practice_1.dot");

  VisibilityTable visibilityTable(filePath);

  EXPECT_DOUBLE_EQ(3.0, visibilityTable.Cost(
    ignition::math::Vector3d(20, 0, -2.5), ignition::math::Vector3d(80, 0, -17.5)));
  EXPECT_DOUBLE_EQ(3.0, visibilityTable.Cost(
    ignition::math::Vector3d(80, 0, -17.5), ignition::math::Vector3d(20, 0, -2.5)));

  std::vector<ignition::math::Vector3d> v =
  {
      {20, 0, -2.5},
      {40, 0, -7.5},
      {60, 0, -12.5},
      {80, 0, -17.5},
      {100, 0, -17.5},
      {120, 0, -17.5},
      {140, 0, -17.5},
      {160, 0, -17.5},
      {180, 0, -17.5},
      {180, 20, -17.5},
      {180, 40, -17.5},
      {160, 40, -17.5},
      {140, 40, -17.5},
      {120, 40, -17.5},
      {100, 40, -17.5},
      {80, 40, -17.5},
      {80, 60, -17.5},
      {80, 80, -17.5},
      {100, 80, -17.5},
      {100, 60, -17.5},
      {100, 20, -17.5},
      {140, 60, -17.5},
      {100, -20, -17.5},
      {120, -20, -17.5},
      {120, -40, -22.5},
      {120, -60, -27.5},
      {120, -80, -27.5},
      {140, -80, -27.5},
      {140, -60, -27.5},
      {140, -40, -27.5},
      {140, -20, -27.5},
      {140, 0, -27.5},
      {160, 0, -27.5},
      {180, 0, -27.5},
      {200, 0, -27.5},
      {220, 0, -32.5},
      {240, 0, -32.5},
      {240, 20, -32.5},
      {160, -80, -27.5},
      {180, -80, -27.5},
      {180, -60, -27.5},
      {180, -40, -27.5},
      {200, -40, -32.5},
      {220, -40, -37.5},
      {240, -40, -37.5},
      {240, -60, -37.5},
      {240, -80, -37.5},
      {-8, 0, 0},
      {-10, -10, 0},
      {-10, 10, 0},
      {10, -10, 0},
      {10, 10, 0},

      {-10, -10, 5},
      {-10, 10, 5},
      {10, -10, 5},
      {10, 10, 5},

      {-10, -10, 10},
      {-10, 10, 10},
      {10, -10, 10},
      {10, 10, 10},

      {-10, -10, 15},
      {-10, 10, 15},
      {10, -10, 15},
      {10, 10, 15},
  };
  for (auto v3d : v)
  {
    std::cout << v3d << ": " << visibilityTable.Index(v3d) << std::endl;
  }
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
