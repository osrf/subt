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
  std::istringstream input(
    "/* Comments should be ignored */ \n"
    "/* <ATTRIBUTE> min_x -500.0 */ \n"
    "/* <ATTRIBUTE> max_x 500.0  */ \n"
    "/* <ATTRIBUTE> step_x 20.0  */ \n"
    "/* <ATTRIBUTE> min_y -500.0 */ \n"
    "/* <ATTRIBUTE> max_y 500.0  */ \n"
    "/* <ATTRIBUTE> step_y 20.0  */ \n"
    "/* <ATTRIBUTE> min_z -50.0  */ \n"
    "/* <ATTRIBUTE> max_z  0     */ \n"
    "/* <ATTRIBUTE> step_z 5.0   */ \n"
    "\n"
    "graph {\n"
    "  0 [label=\"type_0\"] ;\n"
    "  1 [label=\"type_1\"]\n"
    "  2 [label= \"type_2\"];\n"
    "  3 [ label =\"type_3\"];\n\n"
    "  0 -- 1  [label=3.5];\n"
    "  1 -- 2 [  label=1];\n"
    "  2 -- 0 [label= 1 ];\n"
    "}");

  SimpleDOTParser dotParser;
  ASSERT_TRUE(dotParser.Parse(input));

  VisibilityGraph graph = dotParser.Graph();
  EXPECT_EQ(4u, graph.Vertices().size());
  EXPECT_EQ(3u, graph.Edges().size());

  auto hiddenAttributes = dotParser.HiddenAttributes();
  EXPECT_EQ(9u, hiddenAttributes.size());
  ASSERT_NE(hiddenAttributes.end(), hiddenAttributes.find("min_x"));
  EXPECT_EQ("-500.0", hiddenAttributes["min_x"]);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
