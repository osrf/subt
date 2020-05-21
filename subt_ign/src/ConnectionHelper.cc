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


#include <ignition/common/Console.hh>

#include "ConnectionHelper.hh"

std::map<std::string, std::vector<ignition::math::Vector3d>>
  subt::ConnectionHelper::connectionPoints =
  {
    {"Urban Straight", {{0, 20, 0}, {0, -20, 0}}},
    {"Urban Straight Lights", {{0, 20, 0}, {0, -20, 0}}},
    {"Urban Bend Right", {{0, -20, 0}, {20, 0, 0}}},
    {"Urban Bend Left", {{0, -20, 0}, {-20, 0, 0}}},
    {"Urban Bend Left Lights", {{0, -20, 0}, {-20, 0, 0}}},
    {"Urban Superpose", {{0, 20, 0}, {0, -20, 0}, {-20, 0, 10}, {20, 0, 10}}},
    {"Urban 3-Way Right Intersection", {{0, 20, 0}, {0, -20, 0}, {20, 0, 0}}},
    {"Urban Straight Door Right",
        {{20, 0, 0}, {-20, 0, 0}, {-16.021, 3.94, 0.94}}},
    {"Urban Straight Door Left",
        {{20, 0, 0}, {-20, 0, 0}, {-16.021, -3.94, 0.94}}},
    {"Urban Straight Door Right Flipped",
        {{20, 0, 0}, {-20, 0, 0}, {-16.021, 3.94, 0.94}}},
    {"Urban Straight Door Right Flipped Lights",
        {{20, 0, 0}, {-20, 0, 0}, {-16.021, 3.94, 0.94}}},
    {"Urban Straight Door Left Flipped",
        {{20, 0, 0}, {-20, 0, 0}, {-16.021, -3.94, 0.94}}},
    {"Urban Straight Door Right Extension",
        {{20, 0, 0}, {-20, 0, 0}, {0, 20, 0}}},
    {"Urban Straight Door Right Extension Lights",
        {{20, 0, 0}, {-20, 0, 0}, {0, 20, 0}}},
    {"Urban Service Room Centered", {{0, 20, 0}}},
    {"Urban Service Room Centered Lights", {{0, 20, 0}}},
    {"Urban Service Room", {{-16.023, 3.906, 0.919}}},
    {"Urban Service Room Lights", {{-16.023, 3.906, 0.919}}},
    {"Urban Service Room Straight", {{0, -20, 0}, {0, 20, 0}}},
    {"Urban Service Room Straight Lights", {{0, -20, 0}, {0, 20, 0}}},
    {"Urban Platform", {{20, 0, 0}, {-20, 0, 0}, {0, 20, 1.7},
        {23.979, 3.906, 0.94}, {-23.979, 3.906, 0.94}}},
    {"Urban Platform Open", {{20, 0, 0}, {-20, 0, 0}, {0, 20, 0},
        {23.979, 3.906, 0.919}, {-23.979, 3.906, 0.919},
        {23.982, 11.743, 0.919}}},
    {"Urban Stairwell Platform", {{0, 20, 11.69}, {0, -20, 1.69}}},
    {"Urban Stairwell Platform Lights", {{0, 20, 11.69}, {0, -20, 1.69}}},
    {"Urban Stairwell Platform Centered", {{0, 20, 10}, {0, -20, 0}}},
    {"Urban Stairwell Platform Centered Lights", {{0, 20, 10}, {0, -20, 0}}},
    {"Urban Starting Area", {{-16.021, 3.94, 0.919}}},
    {"Urban Elevation Up", {{0, 20, 5}, {0, -20, 0}}},
    {"Urban Elevation Up Lights", {{0, 20, 5}, {0, -20, 0}}},
    {"Urban Elevation Down", {{0, 20, 0}, {0, -20, 5}}},
    {"Urban 2 Story", {{0, 20, 10}, {0, -20, 0}, {-20, 0, 10}, {20, 0, 0}}},
    {"Urban 2 Story Lights",
        {{0, 20, 10}, {0, -20, 0}, {-20, 0, 10}, {20, 0, 0}}},
    {"Urban 2 Story Large Side 1 Lights", {{0, 20, 10}, {0, -20, 0}}},
    {"Urban 2 Story Large Side 2 Lights", {{0, -20, 0}, {0, 20, 0}}},
    {"Urban Large Room Split", {{0, -20, 0}, {-20, 0, 0}, {0, 20, 0}}},
    {"Urban Large Room Split Lights", {{0, -20, 0}, {-20, 0, 0}, {0, 20, 0}}},
    {"Cave Starting Area Type B", {{12.5, 0, 0}}},
    {"Cave Straight 01 Type B", {{0, 12.5, 0}, {0, -12.5, 0}}},
    {"Cave Straight 01 Lights Type B", {{0, 12.5, 0}, {0, -12.5, 0}}},
    {"Cave Straight 02 Type B", {{0, 12.5, 0}, {0, -12.5, 0}}},
    {"Cave Straight 02 Lights Type B", {{0, 12.5, 0}, {0, -12.5, 0}}},
    {"Cave Straight 03 Type B", {{0, 12.5, 0}, {0, -12.5, 0}}},
    {"Cave Straight 04 Type B", {{0, 12.5, 0}, {0, -12.5, 0}}},
    {"Cave Straight 04 Lights Type B", {{0, 12.5, 0}, {0, -12.5, 0}}},
    {"Cave Straight 05 Type B", {{0, 12.5, 0}, {0, -12.5, 0}}},
    {"Cave Straight 05 Lights Type B", {{0, 12.5, 0}, {0, -12.5, 0}}},
    {"Cave Corner 01 Type B", {{-12.5, 0, 0}, {0, 12.5, 0}}},
    {"Cave Corner 01 Lights Type B", {{-12.5, 0, 0}, {0, 12.5, 0}}},
    {"Cave Corner 02 Type B", {{-12.5, 0, 0}, {0, 12.5, 0}}},
    {"Cave Corner 02 Lights Type B", {{-12.5, 0, 0}, {0, 12.5, 0}}},
    {"Cave 3 Way 01 Type B", {{12.5, 0, 0}, {-12.5, 0, 0}, {0, 12.5, 0}}},
    {"Cave 3 Way 01 Lights Type B", {{12.5, 0, 0}, {-12.5, 0, 0}, {0, 12.5, 0}}},
    {"Cave Elevation Type B", {{0, 12.5, 0}, {0, -12.5, 10}}},
    {"Cave Elevation Lights Type B", {{0, 12.5, 0}, {0, -12.5, 10}}},
    {"Cave Vertical Shaft Type B", {{0, 12.5, 20}, {0, -12.5, 0}}},
    {"Cave Vertical Shaft Lights Type B", {{0, 12.5, 20}, {0, -12.5, 0}}},
    {"Cave Cavern Split 01 Type B", {{0, 25, 25}, {12.5, 0, 0}, {-12.5, 0, 0}}},
    {"Cave Cavern Split 02 Type B", {{12.5, 0, 0}, {-12.5, 0, 0}}},
    {"Cave Corner 30 Type B", {{12.5, 0, 0}, {0, 12.5, 0}}},
    {"Cave Corner 30F Type B", {{-12.5, 0, 0}, {0, 12.5, 0}}},
    {"Cave Corner 30 D Type B", {{12.5, 0, 0}, {0, 12.5, 0}}},
    {"Cave Corner 30 D Lights Type B", {{12.5, 0, 0}, {0, 12.5, 0}}},
    {"Cave Corner 30F D Type B", {{12.5, 0, 0}, {0, -12.5, 0}}},
    {"Cave Corner 30F D Lights Type B", {{12.5, 0, 0}, {0, -12.5, 0}}},

    {"Cave 2 Way 01 Type A", {{-25.0, 0.0, 0.0}, {25.0, 0.0, 0.0}}},
    {"Cave 3 Way 01 Type A", {{0.0, -25.0, 0.0}, {-25.0, 50.0, 0.0}, {50.0, 25.0, 0.0}}},
    {"Cave 3 Way 02 Type A", {{-50.0, 25.0, 0.0}, {25.0, 50.0, 0.0}, {0.0, -25.0, 0.0}}},
    {"Cave 3 Way Elevation 01 Type A", {{0.0, 50.0, 0.0}, {-100.0, -50.0, -25.0}, {0.0, -50.0, -25.0}}},
    {"Cave 3 Way Elevation 02 Type A", {{0.0, -50.0, 0.0}, {-100.0, -50.0, -25.0}, {0.0, 50.0, 0.0}}},
    {"Cave 3 Way Elevation 03 Type A", {{0.0, 50.0, 75.0}, {50.0, 0.0, -50.0}, {0.0, -50.0, 0.0}}},
    {"Cave 4 Way 01 Type A", {{0.0, 25.0, 0.0}, {25.0, 0.0, 0.0}, {0.0, -25.0, 0.0}, {-25.0, 0.0, 0.0}}},
    {"Cave Cavern Type A", {{-25.0, 0.0, 25.0}, {25.0, 0.0, 25.0}, {0.0, 25.0, 0.0}, {0.0, -25.0, 0.0}}},
    {"Cave Corner 01 Type A", {{25.0, 0.0, 0.0}, {0.0, -25.0, 0.0}}},
    {"Cave Corner 02 Type A", {{0.0, 25.0, 0.0}, {25.0, -50.0, 0.0}}},
    {"Cave Corner 03 Type A", {{-25.0, 0.0, 0.0}, {0.0, -25.0, 0.0}}},
    {"Cave Corner 04 Type A", {{0.0, 50.0, 25.0}, {-50.0, -50.0, 0.0}}},
    {"Cave Elevation 01 Type A", {{0.0, -50.0, 0.0}, {0.0, 50.0, 75.0}}},
    {"Cave Elevation 02 Type A", {{-50.0, 0.0, 75.0}, {50.0, 0.0, 0.0}}},
    {"Cave Elevation Corner Type A", {{25.0, 75.0, 25.0}, {0.0, -100.0, 0.0}}},
    {"Cave Elevation Straight Type A", {{0.0, 50.0, 25.0}, {0.0, -50.0, 0.0}}},
    {"Cave Split Type A", {{0.0, -50.0, 0.0}, {0.0, 50.0, 0.0}}},
    {"Cave Straight Shift Type A", {{-25.0, 25.0, 0.0}, {25.0, -25.0, 0.0}}},
    {"Cave Straight Type A", {{0.0, 25.0, 0.0}, {0.0, -25.0, 0.0}}},
    {"Cave U Turn 01 Type A", {{-25.0, -25.0, 0.0}, {25.0, -25.0, 0.0}}},
    {"Cave U Turn Elevation Type A", {{-50.0, -25.0, 0.0}, {0.0, -25.0, 25.0}}},
    {"Cave Vertical Shaft Cantilevered Type A", {{0.0, -5.46392e-07, 10.0}, {0.0, 50.0, 25.0}}},
    {"Cave Vertical Shaft Straight Bottom Type A", {{0.0, 0.0, 10.0}, {0.0, 25.0, 0.0}, {0.0, -25.0, 0.0}}},
    {"Cave Vertical Shaft Straight Top Type A", {{0.0, 25.0, 0.0}, {0.0, -25.0, 0.0}, {0.0, 0.0, 0.0}}},
    {"Cave Vertical Shaft Type A", {{0.0, 0.0, 10.0}, {0.0, 0.0, 25.0}}},
    {"Cave Vertical Shaft Dead End Type A", {{0.0, 0.0, 10.0}}},

    {"Cave 3 Way Elevation 02 Lights Type A", {{0.0, -50.0, 0.0}, {-100.0, -50.0, -25.0}, {0.0, 50.0, 0.0}}},
    {"Cave 4 Way 01 Lights Type A", {{0.0, 25.0, 0.0}, {25.0, 0.0, 0.0}, {0.0, -25.0, 0.0}, {-25.0, 0.0, 0.0}}},
    {"Cave Corner 01 Lights Type A", {{25.0, 0.0, 0.0}, {0.0, -25.0, 0.0}}},
    {"Cave Corner 02 Lights Type A", {{0.0, 25.0, 0.0}, {25.0, -50.0, 0.0}}},
    {"Cave Corner 04 Lights Type A", {{0.0, 50.0, 25.0}, {-50.0, -50.0, 0.0}}},
    {"Cave Elevation Straight Lights Type A", {{0.0, 50.0, 25.0}, {0.0, -50.0, 0.0}}},
    {"Cave Straight Lights Type A", {{0.0, 25.0, 0.0}, {0.0, -25.0, 0.0}}},
    {"Cave U Turn Elevation Lights Type A", {{-50.0, -25.0, 0.0}, {0.0, -25.0, 25.0}}},
    {"Cave Vertical Shaft Straight Bottom Lights Type A", {{0.0, 0.0, 10.0}, {0.0, 25.0, 0.0}, {0.0, -25.0, 0.0}}},
    {"Cave Transition Type A to and from Type B", {{0.0, 25.0, 0.0}, {0.0, -25.0, 0.0}}},
    {"Cave Transition Type A to and from Type B Lights", {{0.0, 25.0, 0.0}, {0.0, -25.0, 0.0}}}
  };

std::map<std::string, subt::ConnectionHelper::ConnectionType>
  subt::ConnectionHelper::connectionTypes =
  {
    {"Urban Straight", subt::ConnectionHelper::STRAIGHT},
    {"Urban Straight Lights", subt::ConnectionHelper::STRAIGHT},
    {"Urban Bend Right", subt::ConnectionHelper::TURN},
    {"Urban Bend Left", subt::ConnectionHelper::TURN},
    {"Urban Bend Left Lights", subt::ConnectionHelper::TURN},
    {"Urban Superpose", subt::ConnectionHelper::TURN},
    {"Urban 3-Way Right Intersection", subt::ConnectionHelper::TURN},
    {"Urban Straight Door Right", subt::ConnectionHelper::STRAIGHT},
    {"Urban Straight Door Left", subt::ConnectionHelper::STRAIGHT},
    {"Urban Straight Door Right Flipped", subt::ConnectionHelper::STRAIGHT},
    {"Urban Straight Door Right Flipped Lights",
        subt::ConnectionHelper::STRAIGHT},
    {"Urban Straight Door Left Flipped", subt::ConnectionHelper::STRAIGHT},
    {"Urban Straight Door Right Extension", subt::ConnectionHelper::TURN},
    {"Urban Straight Door Right Extension Lights",
        subt::ConnectionHelper::TURN},
    {"Urban Service Room Centered", subt::ConnectionHelper::STRAIGHT},
    {"Urban Service Room Centered Lights", subt::ConnectionHelper::STRAIGHT},
    {"Urban Service Room", subt::ConnectionHelper::STRAIGHT},
    {"Urban Service Room Lights", subt::ConnectionHelper::STRAIGHT},
    {"Urban Service Room Straight", subt::ConnectionHelper::STRAIGHT},
    {"Urban Service Room Straight Lights", subt::ConnectionHelper::STRAIGHT},
    {"Urban Platform", subt::ConnectionHelper::TURN},
    {"Urban Platform Open", subt::ConnectionHelper::TURN},
    {"Urban Stairwell Platform", subt::ConnectionHelper::STRAIGHT},
    {"Urban Stairwell Platform Lights", subt::ConnectionHelper::STRAIGHT},
    {"Urban Stairwell Platform Centered", subt::ConnectionHelper::STRAIGHT},
    {"Urban Stairwell Platform Centered Lights",
      subt::ConnectionHelper::STRAIGHT},
    {"Urban Starting Area", subt::ConnectionHelper::STRAIGHT},
    {"Urban Elevation Up", subt::ConnectionHelper::STRAIGHT},
    {"Urban Elevation Up Lights", subt::ConnectionHelper::STRAIGHT},
    {"Urban Elevation Down", subt::ConnectionHelper::STRAIGHT},
    {"Urban 2 Story", subt::ConnectionHelper::TURN},
    {"Urban 2 Story Lights", subt::ConnectionHelper::TURN},
    {"Urban 2 Story Large Side 1 Lights", subt::ConnectionHelper::TURN},
    {"Urban 2 Story Large Side 2 Lights", subt::ConnectionHelper::TURN},
    {"Urban Large Room Split", subt::ConnectionHelper::TURN},
    {"Urban Large Room Split Lights", subt::ConnectionHelper::TURN},
    {"Cave Starting Area Type B", subt::ConnectionHelper::STRAIGHT},
    {"Cave Straight 01 Type B", subt::ConnectionHelper::STRAIGHT},
    {"Cave Straight 01 Lights Type B", subt::ConnectionHelper::STRAIGHT},
    {"Cave Straight 02 Type B", subt::ConnectionHelper::STRAIGHT},
    {"Cave Straight 02 Lights Type B", subt::ConnectionHelper::STRAIGHT},
    {"Cave Straight 03 Type B", subt::ConnectionHelper::STRAIGHT},
    {"Cave Straight 04 Type B", subt::ConnectionHelper::STRAIGHT},
    {"Cave Straight 04 Lights Type B", subt::ConnectionHelper::STRAIGHT},
    {"Cave Straight 05 Type B", subt::ConnectionHelper::STRAIGHT},
    {"Cave Straight 05 Lights Type B", subt::ConnectionHelper::STRAIGHT},
    {"Cave Corner 01 Type B", subt::ConnectionHelper::TURN},
    {"Cave Corner 01 Lights Type B", subt::ConnectionHelper::TURN},
    {"Cave Corner 02 Type B", subt::ConnectionHelper::TURN},
    {"Cave Corner 02 Lights Type B", subt::ConnectionHelper::TURN},
    {"Cave 3 Way 01 Type B", subt::ConnectionHelper::TURN},
    {"Cave 3 Way 01 Lights Type B", subt::ConnectionHelper::TURN},
    {"Cave Elevation Type B", subt::ConnectionHelper::STRAIGHT},
    {"Cave Elevation Lights Type B", subt::ConnectionHelper::STRAIGHT},
    {"Cave Vertical Shaft Type B", subt::ConnectionHelper::STRAIGHT},
    {"Cave Vertical Shaft Lights Type B", subt::ConnectionHelper::STRAIGHT},
    {"Cave Cavern Split 01 Type B", subt::ConnectionHelper::TURN},
    {"Cave Cavern Split 02 Type B", subt::ConnectionHelper::STRAIGHT},
    {"Cave Corner 30 Type B", subt::ConnectionHelper::TURN},
    {"Cave Corner 30F Type B", subt::ConnectionHelper::TURN},
    {"Cave Corner 30 D Type B", subt::ConnectionHelper::TURN},
    {"Cave Corner 30 D Lights Type B", subt::ConnectionHelper::TURN},
    {"Cave Corner 30F D Type B", subt::ConnectionHelper::TURN},
    {"Cave Corner 30F D Lights Type B", subt::ConnectionHelper::TURN},

    {"Cave 2 Way 01 Type A", subt::ConnectionHelper::STRAIGHT},
    {"Cave 3 Way 01 Type A", subt::ConnectionHelper::TURN},
    {"Cave 3 Way 02 Type A", subt::ConnectionHelper::TURN},
    {"Cave 3 Way Elevation 01 Type A", subt::ConnectionHelper::TURN},
    {"Cave 3 Way Elevation 02 Type A", subt::ConnectionHelper::TURN},
    {"Cave 3 Way Elevation 03 Type A", subt::ConnectionHelper::TURN},
    {"Cave 4 Way 01 Type A", subt::ConnectionHelper::TURN},
    {"Cave Cavern Type A", subt::ConnectionHelper::TURN},
    {"Cave Corner 01 Type A", subt::ConnectionHelper::TURN},
    {"Cave Corner 02 Type A", subt::ConnectionHelper::TURN},
    {"Cave Corner 03 Type A", subt::ConnectionHelper::TURN},
    {"Cave Corner 04 Type A", subt::ConnectionHelper::TURN},
    {"Cave Elevation 01 Type A", subt::ConnectionHelper::STRAIGHT},
    {"Cave Elevation 02 Type A", subt::ConnectionHelper::STRAIGHT},
    {"Cave Elevation Corner Type A", subt::ConnectionHelper::TURN},
    {"Cave Elevation Straight Type A", subt::ConnectionHelper::TURN},
    {"Cave Split Type A", subt::ConnectionHelper::TURN},
    {"Cave Straight Shift Type A", subt::ConnectionHelper::STRAIGHT},
    {"Cave Straight Type A", subt::ConnectionHelper::STRAIGHT},
    {"Cave U Turn 01 Type A", subt::ConnectionHelper::TURN},
    {"Cave U Turn Elevation Type A", subt::ConnectionHelper::TURN},
    {"Cave Vertical Shaft Cantilevered Type A", subt::ConnectionHelper::TURN},
    {"Cave Vertical Shaft Straight Bottom Type A", subt::ConnectionHelper::TURN},
    {"Cave Vertical Shaft Straight Top Type A", subt::ConnectionHelper::TURN},
    {"Cave Vertical Shaft Type A", subt::ConnectionHelper::STRAIGHT},
    {"Cave Vertical Shaft Dead End Type A", subt::ConnectionHelper::STRAIGHT},
    {"Cave 3 Way Elevation 02 Lights Type A", subt::ConnectionHelper::TURN},
    {"Cave 4 Way 01 Lights Type A", subt::ConnectionHelper::TURN},
    {"Cave Corner 01 Lights Type A", subt::ConnectionHelper::TURN},
    {"Cave Corner 02 Lights Type A", subt::ConnectionHelper::TURN},
    {"Cave Corner 04 Lights Type A", subt::ConnectionHelper::TURN},
    {"Cave Elevation Straight Lights Type A", subt::ConnectionHelper::TURN},
    {"Cave Straight Lights Type A", subt::ConnectionHelper::STRAIGHT},
    {"Cave U Turn Elevation Lights Type A", subt::ConnectionHelper::TURN},
    {"Cave Vertical Shaft Straight Bottom Lights Type A", subt::ConnectionHelper::TURN},
    {"Cave Transition Type A to and from Type B", subt::ConnectionHelper::STRAIGHT},
    {"Cave Transition Type A to and from Type B Lights", subt::ConnectionHelper::STRAIGHT}
  };

using namespace ignition;
using namespace subt;

/////////////////////////////////////////////////
bool ConnectionHelper::ComputePoint(VertexData *_tile1, VertexData *_tile2,
    ignition::math::Vector3d& _pt)
{
  if (!ConnectionHelper::connectionPoints.count(_tile1->tileType))
  {
    ignwarn << "No connection information for: " << _tile1->tileType
            << std::endl;
    return false;
  }

  if (!ConnectionHelper::connectionPoints.count(_tile2->tileType))
  {
    ignwarn << "No connection information for: " << _tile2->tileType
            << std::endl;
    return false;
  }

  for (const auto& pt1 : ConnectionHelper::connectionPoints[_tile1->tileType])
  {
    auto pt1tf = _tile1->model.RawPose().CoordPositionAdd(pt1);
    for (const auto& pt2 : ConnectionHelper::connectionPoints[_tile2->tileType])
    {
      auto pt2tf = _tile2->model.RawPose().CoordPositionAdd(pt2);
      if (pt1tf.Equal(pt2tf, 1))
      {
        _pt = pt1tf;
        return true;
      }
    }
  }

  ignwarn << "Failed to connect: " << _tile1->tileType << " "
          << _tile2->tileType << std::endl;

  for (const auto& pt1 : ConnectionHelper::connectionPoints[_tile1->tileType])
  {
    auto pt1tf = _tile1->model.RawPose().CoordPositionAdd(pt1);
    for (const auto& pt2 :
        ConnectionHelper::connectionPoints[_tile2->tileType])
    {
      auto pt2tf = _tile2->model.RawPose().CoordPositionAdd(pt2);
      igndbg <<
        _tile1->tileType << " [" << _tile1->model.RawPose() << "] -- " <<
        _tile2->tileType << " [" << _tile2->model.RawPose() << "]"
                         << " [" <<  pt1tf << "] [" << pt2tf << "]"
                         << std::endl;
    }
  }

  return false;
}

/////////////////////////////////////////////////
std::vector<ignition::math::Vector3d> ConnectionHelper::GetConnectionPoints(VertexData *_tile1)
{
  auto ret = std::vector<ignition::math::Vector3d>();

  if (!ConnectionHelper::connectionPoints.count(_tile1->tileType))
  {
    ignwarn << "No connection information for: " << _tile1->tileType
            << std::endl;
  }
  else
  {
    for (const auto& pt1 : ConnectionHelper::connectionPoints[_tile1->tileType])
    {
      auto pt1tf = _tile1->model.Pose().CoordPositionAdd(pt1);
      ret.push_back(pt1tf);
    }
  }
  return ret;
}
