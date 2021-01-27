/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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


#include "subt_ign/VisibilityTable.hh"

using VisibilityTable = subt::VisibilityTable;

int main(int argc, char **argv)
{
  if (argc != 2)
  {
    std::cerr << "Usage run_visibility_table <world>" << std::endl << std::endl;
    std::cerr << "Example: ./run_visibility_table simple_cave_02" << std::endl;
    return -1;
  }

  VisibilityTable visibilityTable;
  visibilityTable.Load(argv[1], true);

  // Uncomment for checking visibility table information. E.g.:

  // auto tileId = visibilityTable.Index({90, 2.42, 0.26});
  // auto tileId = visibilityTable.Index({29.59, 2.36, 0.77});
  // std::cout << "Tile id: " << tileId << std::endl;

  // Add relays. E.g.:
  // visibilityTable.PopulateVisibilityInfo(
  // {
  //   {26.67, 103.91, 1.2}, // #11
  //   {50, 125, 2},         // #3
  //   {100, 125, 0},        // #5
  // });

  return 0;
}

