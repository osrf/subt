/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <subt_ign/Config.hh>
#include <subt_ign/Common.hh>
#include <subt_ign/CommonTypes.hh>

#include <ros/package.h>

/////////////////////////////////////////////////
TEST(subt_ign_Common, ArtifactTypes){
  // Tests to make sure all of our types are consistent
  //
  // It is expected that these tests will fail when artifacts
  // are added or removed, and should be adjusted as such.
  auto num_artifacts = static_cast<uint32_t>(subt::ArtifactType::Count);
  ASSERT_EQ(14u, num_artifacts);
  ASSERT_EQ(num_artifacts, subt::kArtifactNames.size());
  ASSERT_EQ(num_artifacts, subt::kArtifactTypes.size());
}

/////////////////////////////////////////////////
TEST(subt_ign_Common, ArtifactFromInt){
  // Hardcoding to flag if definition changes
  std::vector<std::pair<uint32_t, subt::ArtifactType>> expected = {
    {0, subt::ArtifactType::TYPE_BACKPACK},
    {1, subt::ArtifactType::TYPE_DRILL},
    {2, subt::ArtifactType::TYPE_DUCT},
    {3, subt::ArtifactType::TYPE_ELECTRICAL_BOX},
    {4, subt::ArtifactType::TYPE_EXTINGUISHER},
    {5, subt::ArtifactType::TYPE_PHONE},
    {6, subt::ArtifactType::TYPE_RADIO},
    {7, subt::ArtifactType::TYPE_RESCUE_RANDY},
    {8, subt::ArtifactType::TYPE_TOOLBOX},
    {9, subt::ArtifactType::TYPE_VALVE},
    {10, subt::ArtifactType::TYPE_VENT},
    {11, subt::ArtifactType::TYPE_GAS},
    {12, subt::ArtifactType::TYPE_HELMET},
    {13, subt::ArtifactType::TYPE_ROPE},
    {14, subt::ArtifactType::TYPE_CUBE}
  };

  for (const auto &[input, expected_out] : expected)
  {
    subt::ArtifactType type;
    EXPECT_TRUE(subt::ArtifactFromInt(input, type));
    EXPECT_EQ(expected_out, type);
  }

  {
    // Test garbage input
    subt::ArtifactType type;
    EXPECT_FALSE(subt::ArtifactFromInt(-1, type));
  }

  {
    // This test will fail if artifacts are added
    subt::ArtifactType type;
    EXPECT_FALSE(subt::ArtifactFromInt(14, type));
  }
}

/////////////////////////////////////////////////
TEST(subt_ign_Common, ArtifactFromString){
  std::vector<std::pair<std::string, subt::ArtifactType>> expected = {
    {"TYPE_BACKPACK", subt::ArtifactType::TYPE_BACKPACK},
    {"TYPE_DRILL", subt::ArtifactType::TYPE_DRILL},
    {"TYPE_DUCT", subt::ArtifactType::TYPE_DUCT},
    {"TYPE_ELECTRICAL_BOX", subt::ArtifactType::TYPE_ELECTRICAL_BOX},
    {"TYPE_EXTINGUISHER", subt::ArtifactType::TYPE_EXTINGUISHER},
    {"TYPE_PHONE", subt::ArtifactType::TYPE_PHONE},
    {"TYPE_RADIO", subt::ArtifactType::TYPE_RADIO},
    {"TYPE_RESCUE_RANDY", subt::ArtifactType::TYPE_RESCUE_RANDY},
    {"TYPE_TOOLBOX", subt::ArtifactType::TYPE_TOOLBOX},
    {"TYPE_VALVE", subt::ArtifactType::TYPE_VALVE},
    {"TYPE_VENT", subt::ArtifactType::TYPE_VENT},
    {"TYPE_GAS", subt::ArtifactType::TYPE_GAS},
    {"TYPE_HELMET", subt::ArtifactType::TYPE_HELMET},
    {"TYPE_ROPE", subt::ArtifactType::TYPE_ROPE},
    {"TYPE_CUBE", subt::ArtifactType::TYPE_CUBE}
  };

  for (const auto &[input, expected_out] : expected)
  {
    subt::ArtifactType type;
    EXPECT_TRUE(subt::ArtifactFromString(input, type));
    EXPECT_EQ(expected_out, type);
  }

  {
    // Returns false on unknown string
    subt::ArtifactType type;
    EXPECT_FALSE(subt::ArtifactFromString("foobar", type));
  }
}

/////////////////////////////////////////////////
TEST(subt_ign_Common, ArtifactFromPartialString){
  {
    subt::ArtifactType type;
    EXPECT_TRUE(subt::ArtifactFromPartialString("backpack_1", type));
    EXPECT_EQ(subt::ArtifactType::TYPE_BACKPACK, type);
  }
  {
    subt::ArtifactType type;
    EXPECT_TRUE(subt::ArtifactFromPartialString("rope_1", type));
    EXPECT_EQ(subt::ArtifactType::TYPE_ROPE, type);
  }
}

/////////////////////////////////////////////////
TEST(subt_ign_Common, StringFromArtifact) {
  {
    auto type = subt::ArtifactType::TYPE_BACKPACK;
    std::string output;
    EXPECT_TRUE(subt::StringFromArtifact(type, output));
    EXPECT_EQ("TYPE_BACKPACK", output);
  }
  {
    auto type = subt::ArtifactType::TYPE_ROPE;
    std::string output;
    EXPECT_TRUE(subt::StringFromArtifact(type, output));
    EXPECT_EQ("TYPE_ROPE", output);
  }
}

/////////////////////////////////////////////////
TEST(subt_ign_Common, FullWorldPath) {
  std::vector<std::pair<std::string, std::string>> expected = {
    {"simple_cave_01", "simple_cave_01"},
    {"cave_qual", "cave_qual"},
    {"cave_circuit_practice_01", "cave_circuit_practice_01"},
    {"cave_circuit_01", "cave_circuit/01/cave_circuit_01"},

    {"simple_tunnel_02", "simple_tunnel_02"},
    {"tunnel_circuit_practice_02", "tunnel_circuit_practice_02"},
    {"tunnel_qual_ign", "tunnel_qual_ign"},
    {"tunnel_circuit_02", "tunnel_circuit/02/tunnel_circuit_02"},

    {"simple_urban_03", "simple_urban_03"},
    {"urban_circuit_practice_03", "urban_circuit_practice_03"},
    {"urban_qual", "urban_qual"},
    {"urban_circuit_03", "urban_circuit/03/urban_circuit_03"},
  };

  std::string worldsDirectory = ignition::common::joinPaths(
    ros::package::getPath("subt_ign"), "worlds");
  for (const auto &[input, expected_out] : expected)
  {
    std::string worldPath;
    EXPECT_TRUE(subt::FullWorldPath(input, worldPath));
    EXPECT_EQ(
        ignition::common::joinPaths(worldsDirectory, expected_out),
        worldPath);
  }

  {
    std::string worldPath;
    EXPECT_FALSE(subt::FullWorldPath("foobar", worldPath));
  }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

