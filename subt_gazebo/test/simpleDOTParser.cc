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
#include <sstream>
#include "subt_gazebo/SimpleDOTParser.hh"

using namespace subt;

/////////////////////////////////////////////////
TEST(SimpleDOTParserTest, StreamExtraction)
{
  SimpleDOTParser dotParser;
  VisibilityGraph graph;
  std::istringstream input(
    "graph {\n"
    "  0 [label=\"type 0\"];\n"
    "  1 [label=\"type 1\"];\n"
    "  2 [label=\"type 2\"];\n"
    "  3 [label=\"type 3\"];\n\n"
    "  0 -- 1 [label=4];\n"
    "  1 -- 2 [label=1];\n"
    "  2 -- 0 [label=1];\n"
    "}");

  ASSERT_TRUE(dotParser.Parse(input, graph));
  EXPECT_EQ(4u, graph.Vertices().size());
  EXPECT_EQ(3u, graph.Edges().size());
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
