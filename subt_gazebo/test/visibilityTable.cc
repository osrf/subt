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
      {10, -10, -2.5},
      {30, -10, -7.5},
      {50, -10, -12.5},
      {70, -10, -17.5},
      {90, -10, -17.5},
      {110, -10, -17.5},
      {130, -10, -17.5},
      {150, -10, -17.5},
      {170, -10, -17.5},
      {170, 10, -17.5},
      {170, 30, -17.5},
      {150, 30, -17.5},
      {130, 30, -17.5},
      {110, 30, -17.5},
      {90, 30, -17.5},
      {70, 30, -17.5},
      {70, 50, -17.5},
      {70, 70, -17.5},
      {90, 70, -17.5},
      {90, 50, -17.5},
      {90, 10, -17.5},
      {130, 50, -17.5},
      {90, -30, -17.5},
      {110, -30, -17.5},
      {110, -50, -22.5},
      {110, -70, -27.5},
      {110, -90, -27.5},
      {130, -90, -27.5},
      {130, -70, -27.5},
      {130, -50, -27.5},
      {130, -30, -27.5},
      {130, -10, -27.5},
      {150, -10, -27.5},
      {170, -10, -27.5},
      {190, -10, -27.5},
      {210, -10, -32.5},
      {230, -10, -32.5},
      {230,  10, -32.5},
      {150, -70, -27.5},
      {170, -70, -27.5},
      {170, -50, -27.5},
      {170, -30, -27.5},
      {190, -30, -32.5},
      {210, -30, -37.5},
      {230, -30, -37.5},
      {230, -50, -37.5},
      {230, -70, -37.5},
      {2,   -10, -2.5},
      
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
