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
    ignition::math::Vector3d(20, 0, -5), ignition::math::Vector3d(80, 0, -20)));
  EXPECT_DOUBLE_EQ(3.0, visibilityTable.Cost(
    ignition::math::Vector3d(80, 0, -20), ignition::math::Vector3d(20, 0, -5)));
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
