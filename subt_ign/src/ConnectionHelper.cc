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
    {"Constrained Tunnel Tile Short", {{0, -10, 0}, {0, 10, 0}}},
    {"Constrained Tunnel Tile Tall", {{0, -10, 0}, {0, 10, 0}}},
    {"Tunnel Bend Right", {{0, 0, 0}, {15, 25, 0}}},
    {"Tunnel Corner Left", {{0, 0, 0}, {-10, 15, 0}}},
    {"Tunnel Corner Right", {{0, 0, 0}, {10, 15, 0}}},
    {"Tunnel Elevation", {{0, 0, 0}, {0, 20, 5}}},
    {"Tunnel Intersection", {{0, 0, 0}, {7.5, 7.5, 0}, {-7.5, 7.5, 0}, {0, 15, 0}}},
    {"Tunnel Intersection T", {{0, 0, 0}, {7.5, 7.5, 0}, {-7.5, 7.5, 0}}},
    {"Tunnel Straight", {{0, 0, 0}, {0, 5, 0}}},
    {"subt_tunnel_staging_area", {{10, 0, 0}}},
    {"Tunnel Tile 1", {{0.0, 10.0, 0.0}, {10.0, 0.0, 0.0}, {0.0, -10.0, 0.0}, {-10.0, 0.0, 0.0}}},
    {"Tunnel Tile 1 Lights", {{0.0, 10.0, 0.0}, {10.0, 0.0, 0.0}, {0.0, -10.0, 0.0}, {-10.0, 0.0, 0.0}}},
    {"Tunnel Tile 2", {{10, 0, 0}, {0, -10, 0}}},
    {"Tunnel Tile 2 Lights", {{10, 0, 0}, {0, -10, 0}}},
    {"Tunnel Tile 3", {{0.0, 10.0, 0.0}, {10.0, 0.0, 0.0}, {0.0, -10.0, 0.0}, {-10.0, 0.0, 0.0}}},
    {"Tunnel Tile 3 Lights", {{0.0, 10.0, 0.0}, {10.0, 0.0, 0.0}, {0.0, -10.0, 0.0}, {-10.0, 0.0, 0.0}}},
    {"Tunnel Tile 4", {{0.0, 10.0, 0.0}, {10.0, 0.0, 0.0}, {0.0, -10.0, 0.0}, {-10.0, 0.0, 0.0}}},
    {"Tunnel Tile 4 Lights", {{0.0, 10.0, 0.0}, {10.0, 0.0, 0.0}, {0.0, -10.0, 0.0}, {-10.0, 0.0, 0.0}}},
    {"Tunnel Tile 5", {{0, -10, 0}, {0, 10, 0}}},
    {"Tunnel Tile 5 Lights", {{0, -10, 0}, {0, 10, 0}}},
    {"Tunnel Tile 6", {{0, -10, 0}, {0, 10, 5}}},
    {"Tunnel Tile 6 Lights", {{0, -10, 0}, {0, 10, 5}}},
    {"Tunnel Tile 7", {{0, -10, 0}, {0, 10, 5}}},
    {"Tunnel Tile 7 Lights", {{0, -10, 0}, {0, 10, 5}}},
    {"Rough Tunnel Tile 90-degree Turn", {{0, -10, 0}, {10, 0, 0}}},
    {"Rough Tunnel Tile Ramp", {{0, -10, 0}, {0, 10, 5}}},
    {"Rough Tunnel Tile Straight", {{0, -10, 0}, {0, 10, 0}}},
    {"Rough Tunnel Tile Vertical Shaft", {{0, -10, 0}, {0, 10, 5}}},
    {"Rough Tunnel Tile 4-way Intersection", {{0, -10, 0}, {0, 10, 0}, {10, 0, 0}, {-10, 0, 0}}},

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
    {"Urban 2 Story Large Side 1", {{0, 20, 10}, {0, -20, 0}}},
    {"Urban 2 Story Large Side 2", {{0, -20, 0}, {0, 20, 0}}},
    {"Urban Large Room Split", {{0, -20, 0}, {-20, 0, 0}, {0, 20, 0}}},
    {"Urban Large Room Split Lights", {{0, -20, 0}, {-20, 0, 0}, {0, 20, 0}}},
    {"Cave Starting Area Type B", {{12.5, 0, 0}}},
    {"Cave Straight 01", {{0, 12.5, 0}, {0, -12.5, 0}}},
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
    {"Cave Corner 03 Type A Lights", {{-25.0, 0.0, 0.0}, {0.0, -25.0, 0.0}}},
    {"Cave Corner 04 Type A", {{0.0, 50.0, 25.0}, {-50.0, -50.0, 0.0}}},
    {"Cave Cap Type A", {{0.0, 0.0, 0.0}}},
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
    {"Cave Transition Type A to and from Type B Lights", {{0.0, 25.0, 0.0}, {0.0, -25.0, 0.0}}},

    {"Jenolan Section 01", {{401.144, 13.2017, 63.5658}}},
    {"Jenolan Section 02", {{401.144, 13.2017, 63.5658}, {231.676, 5.12208, 83.5523}}},
    {"Jenolan Section 03", {{231.676, 5.12208, 83.5523}, {193.48, 39.2667, 62.0974}, {173.354, 3.1715, 83.8528}, {175.939, 15.8454, 74.7845}}},
    {"Jenolan Section 04", {{193.48, 39.2667, 62.0974}, {173.354, 3.1715, 83.8528}, {175.939, 15.8454, 74.7845}, {108.945, -.808552, 117.401}, {102.694, -4.59171, 47.6574}}},
    {"Jenolan Section 05", {{43.2806, -4.87545, 130.876}, {108.945, -.808552, 117.401}, {102.694, -4.59171, 47.6574}, {0, 0, 12.5}}},
    {"Jenolan Section 06", {{95.93, 26.3732, 146.106}, {90.7982, -7.84003, 167.789}, {-16.465, 17.3448, 159.529}, {43.2806, -4.87545, 130.876}}},
    {"Jenolan Section 07", {{95.93, 26.3732, 146.106}, {90.7982, -7.84003, 167.789}}},
    {"Jenolan Section 08", {{-82.6196, 39.5982, 140.961}, {-76.3975, 30.5907, 134.982}, {-71.9846, 10.5912, 139.379}, {-16.465, 17.3448, 159.529}}},
    {"Jenolan Section 09", {{-194.72, -9.770950, 132.04}, {-82.6196, 39.5982, 140.961}, {-76.3975, 30.5907, 134.982}, {-71.9846, 10.5912, 139.379}}},
    {"Jenolan Section 10", {{-264.458, 25.7639, 43.2923}, {-241.489, -6.8719, 112.301}, {-194.72, -9.770950, 132.04}}},
    {"Jenolan Section 11", {{-264.458, 25.7639, 43.2923}, {-241.489, -6.8719, 112.301}}},
    {"Edgar Mine Virtual STIX Staging", {{36.59, -98.89, 0.22}}},
    {"Edgar Mine Virtual STIX 1", {{-5.357750, -3.895970, 1.542500}, {14.732500, -54.425900, 1.309480}, {22.352600, -47.677800, 1.840880}, {36.688300, -99.091400, 0.502152}}},
    {"Edgar Mine Virtual STIX 2", {{-28.023300, -8.152720, 1.523010}, {-45.176900, -15.067200, 2.253390}, {-19.575200, -76.385600, 1.611400}, {6.422350, -64.002200, 1.529890}, {14.710200, -54.129700, 1.529890}}},
    {"Edgar Mine Virtual STIX 3", {{-19.323300, -76.383000, 1.596170}, {-57.390400, -91.166200, 0.836131}, {-43.042400, -105.988000, 1.170930}}},
    {"Edgar Mine Virtual STIX 4", {{-76.194300, -42.517800, 1.360100},{-57.313300, -90.930900, 0.757498}}},
    {"Edgar Mine Virtual STIX 5", {{-5.551090, -3.690460, 1.977060},{-10.253500, 4.161460, 1.977060},{-28.316800, -7.717830, 1.677650},{-45.286200, -14.888900, 2.101940},{-76.032100, -42.339900, 1.440770}}},
    {"Edgar Mine Virtual STIX 6", {{-10.134400, 4.764300, 1.700290}, {-21.417500, 20.110100, 1.645820}, {13.817500, 26.329800, 2.133420}, {61.013200, 34.655400, 2.746590}}},
    {"Edgar Mine Virtual STIX 7", {{-21.210300, 20.283600, 1.809090}, {-57.365300, 62.626300, 1.809090} ,{-58.171400, 76.645800, 1.761010}, {-24.210600, 97.656100, 13.359000}}},
    {"Edgar Mine Virtual STIX 8", {{13.824900, 26.428100, 2.063650}, {-24.076000, 97.654300, 12.870300}}},
    {"Edgar Mine Virtual STIX 9", {{69.842000, -14.046400, 2.208530}, {61.017200, 34.481600, 2.769580} ,{106.348000, 8.531670, 2.551910}}},
    {"Edgar Mine Virtual STIX 10", {{22.650300, -47.697300, 1.670630} ,{69.681400, -14.199900, 2.066190}, {106.374000, 8.160290, 2.428980}}},
    {"Universal Straight 10", {{0, 5, 0}, {0, -5, 0}}},
    {"Universal Straight 5", {{0, 2.5, 0}, {0, -2.5, 0}}},
    {"Universal Straight 2.5", {{0, 1.25, 0}, {0, -1.25, 0}}},
    {"Universal Shift 5x5", {{0, 2.5, 0}, {2.5, -2.5, 0}}},
    {"Universal Shift 2.5x2.5", {{0, 1.25, 0}, {1.25, -1.25, 0}}},
    {"Cave Tunnel Transition", {{0, 25, 0}, {0, -25, 0}, {-25, 0, 0}}},
    {"Cave Tunnel Transition Lights", {{0, 25, 0}, {0, -25, 0}, {-25, 0, 0}}},
    {"Urban Cave Transition", {{0, -12, 0}, {0, 12, 0}, {-25, 0, 0}}},
    {"Urban Cave Transition Straight", {{0, -5, 0}, {0, 5, 0}}},
    {"Urban Tunnel Transition", {{0, 0, 0}, {0, -15, -5}}},
    {"4-Way Finals Transition 2", {{8, 0, 0}, {-8, 0, 0}, {0, 8, 0}, {0, -8, 0}}},
    {"4-Way Finals Transition 2 Lights", {{8, 0, 0}, {-8, 0, 0}, {0, 8, 0}, {0, -8, 0}}},
    {"Finals Staging Area", {{0, 5, 0}}},
  };

std::map<std::string, subt::ConnectionHelper::ConnectionType>
  subt::ConnectionHelper::connectionTypes =
  {
    {"Tunnel Tile 1", subt::ConnectionHelper::TURN},
    {"Tunnel Tile 1 Lights", subt::ConnectionHelper::TURN},
    {"Tunnel Tile 2", subt::ConnectionHelper::TURN},
    {"Tunnel Tile 2 Lights", subt::ConnectionHelper::TURN},
    {"Tunnel Tile 3", subt::ConnectionHelper::TURN},
    {"Tunnel Tile 3 Lights", subt::ConnectionHelper::TURN},
    {"Tunnel Tile 4", subt::ConnectionHelper::TURN},
    {"Tunnel Tile 4 Lights", subt::ConnectionHelper::TURN},
    {"Tunnel Tile 5", subt::ConnectionHelper::STRAIGHT},
    {"Tunnel Tile 5 Lights", subt::ConnectionHelper::STRAIGHT},
    {"Tunnel Tile 6", subt::ConnectionHelper::STRAIGHT},
    {"Tunnel Tile 6 Lights", subt::ConnectionHelper::STRAIGHT},
    {"Tunnel Tile 7", subt::ConnectionHelper::TURN},
    {"Tunnel Tile 7 Lights", subt::ConnectionHelper::TURN},
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
    {"Urban 2 Story Large Side 1", subt::ConnectionHelper::TURN},
    {"Urban 2 Story Large Side 2", subt::ConnectionHelper::TURN},
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
    {"Cave Corner 03 Type A Lights", subt::ConnectionHelper::TURN},
    {"Cave Corner 04 Type A", subt::ConnectionHelper::TURN},
    {"Cave Cap Type A", subt::ConnectionHelper::TURN},
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
    {"Cave Transition Type A to and from Type B Lights", subt::ConnectionHelper::STRAIGHT},

    {"Jenolan Section 01", subt::ConnectionHelper::TURN},
    {"Jenolan Section 02", subt::ConnectionHelper::TURN},
    {"Jenolan Section 03", subt::ConnectionHelper::TURN},
    {"Jenolan Section 04", subt::ConnectionHelper::TURN},
    {"Jenolan Section 05", subt::ConnectionHelper::TURN},
    {"Jenolan Section 06", subt::ConnectionHelper::TURN},
    {"Jenolan Section 07", subt::ConnectionHelper::TURN},
    {"Jenolan Section 08", subt::ConnectionHelper::TURN},
    {"Jenolan Section 09", subt::ConnectionHelper::TURN},
    {"Jenolan Section 10", subt::ConnectionHelper::TURN},
    {"Jenolan Section 11", subt::ConnectionHelper::TURN},
    {"Universal Straight 10", subt::ConnectionHelper::STRAIGHT},
    {"Universal Straight 5", subt::ConnectionHelper::STRAIGHT},
    {"Universal Straight 2.5", subt::ConnectionHelper::STRAIGHT},
    {"Universal Shift 5x5", subt::ConnectionHelper::STRAIGHT},
    {"Universal Shift 2.5x2.5", subt::ConnectionHelper::STRAIGHT},
    {"Cave Tunnel Transition", subt::ConnectionHelper::STRAIGHT},
    {"Cave Tunnel Transition Lights", subt::ConnectionHelper::STRAIGHT},
    {"Urban Cave Transition", subt::ConnectionHelper::STRAIGHT},
    {"Urban Cave Transition Straight", subt::ConnectionHelper::STRAIGHT},
    {"Urban Tunnel Transition", subt::ConnectionHelper::STRAIGHT},
    {"Finals Staging Area", subt::ConnectionHelper::STRAIGHT},
    {"4-Way Finals Transition", subt::ConnectionHelper::TURN},
    {"4-Way Finals Transition 2", subt::ConnectionHelper::TURN},
    {"4-Way Finals Transition 2 Lights", subt::ConnectionHelper::TURN},
  };

  std::map<std::string, subt::ConnectionHelper::CircuitType>
  subt::ConnectionHelper::circuitTypes =
  {
    {"Tunnel Tile 1", subt::ConnectionHelper::TUNNEL},
    {"Tunnel Tile 1 Lights", subt::ConnectionHelper::TUNNEL},
    {"Tunnel Tile 2", subt::ConnectionHelper::TUNNEL},
    {"Tunnel Tile 2 Lights", subt::ConnectionHelper::TUNNEL},
    {"Tunnel Tile 3", subt::ConnectionHelper::TUNNEL},
    {"Tunnel Tile 3 Lights", subt::ConnectionHelper::TUNNEL},
    {"Tunnel Tile 4", subt::ConnectionHelper::TUNNEL},
    {"Tunnel Tile 4 Lights", subt::ConnectionHelper::TUNNEL},
    {"Tunnel Tile 5", subt::ConnectionHelper::TUNNEL},
    {"Tunnel Tile 5 Lights", subt::ConnectionHelper::TUNNEL},
    {"Tunnel Tile 6", subt::ConnectionHelper::TUNNEL},
    {"Tunnel Tile 6 Lights", subt::ConnectionHelper::TUNNEL},
    {"Tunnel Tile 7", subt::ConnectionHelper::TUNNEL},
    {"Tunnel Tile 7 Lights", subt::ConnectionHelper::TUNNEL},
    {"Urban Straight", subt::ConnectionHelper::URBAN},
    {"Urban Straight Lights", subt::ConnectionHelper::URBAN},
    {"Urban Bend Right", subt::ConnectionHelper::URBAN},
    {"Urban Bend Left", subt::ConnectionHelper::URBAN},
    {"Urban Bend Left Lights", subt::ConnectionHelper::URBAN},
    {"Urban Superpose", subt::ConnectionHelper::URBAN},
    {"Urban 3-Way Right Intersection", subt::ConnectionHelper::URBAN},
    {"Urban Straight Door Right", subt::ConnectionHelper::URBAN},
    {"Urban Straight Door Left", subt::ConnectionHelper::URBAN},
    {"Urban Straight Door Right Flipped", subt::ConnectionHelper::URBAN},
    {"Urban Straight Door Right Flipped Lights",
        subt::ConnectionHelper::URBAN},
    {"Urban Straight Door Left Flipped", subt::ConnectionHelper::URBAN},
    {"Urban Straight Door Right Extension", subt::ConnectionHelper::URBAN},
    {"Urban Straight Door Right Extension Lights",
        subt::ConnectionHelper::URBAN},
    {"Urban Service Room Centered", subt::ConnectionHelper::URBAN},
    {"Urban Service Room Centered Lights", subt::ConnectionHelper::URBAN},
    {"Urban Service Room", subt::ConnectionHelper::URBAN},
    {"Urban Service Room Lights", subt::ConnectionHelper::URBAN},
    {"Urban Service Room Straight", subt::ConnectionHelper::URBAN},
    {"Urban Service Room Straight Lights", subt::ConnectionHelper::URBAN},
    {"Urban Platform", subt::ConnectionHelper::URBAN},
    {"Urban Platform Open", subt::ConnectionHelper::URBAN},
    {"Urban Stairwell Platform", subt::ConnectionHelper::URBAN},
    {"Urban Stairwell Platform Lights", subt::ConnectionHelper::URBAN},
    {"Urban Stairwell Platform Centered", subt::ConnectionHelper::URBAN},
    {"Urban Stairwell Platform Centered Lights",
      subt::ConnectionHelper::URBAN},
    {"Urban Starting Area", subt::ConnectionHelper::URBAN},
    {"Urban Elevation Up", subt::ConnectionHelper::URBAN},
    {"Urban Elevation Up Lights", subt::ConnectionHelper::URBAN},
    {"Urban Elevation Down", subt::ConnectionHelper::URBAN},
    {"Urban 2 Story", subt::ConnectionHelper::URBAN},
    {"Urban 2 Story Lights", subt::ConnectionHelper::URBAN},
    {"Urban 2 Story Large Side 1 Lights", subt::ConnectionHelper::URBAN},
    {"Urban 2 Story Large Side 2 Lights", subt::ConnectionHelper::URBAN},
    {"Urban 2 Story Large Side 1", subt::ConnectionHelper::URBAN},
    {"Urban 2 Story Large Side 2", subt::ConnectionHelper::URBAN},
    {"Urban Large Room Split", subt::ConnectionHelper::URBAN},
    {"Urban Large Room Split Lights", subt::ConnectionHelper::URBAN},
    {"Cave Starting Area Type B", subt::ConnectionHelper::CAVE},
    {"Cave Straight 01 Type B", subt::ConnectionHelper::CAVE},
    {"Cave Straight 01 Lights Type B", subt::ConnectionHelper::CAVE},
    {"Cave Straight 02 Type B", subt::ConnectionHelper::CAVE},
    {"Cave Straight 02 Lights Type B", subt::ConnectionHelper::CAVE},
    {"Cave Straight 03 Type B", subt::ConnectionHelper::CAVE},
    {"Cave Straight 04 Type B", subt::ConnectionHelper::CAVE},
    {"Cave Straight 04 Lights Type B", subt::ConnectionHelper::CAVE},
    {"Cave Straight 05 Type B", subt::ConnectionHelper::CAVE},
    {"Cave Straight 05 Lights Type B", subt::ConnectionHelper::CAVE},
    {"Cave Corner 01 Type B", subt::ConnectionHelper::CAVE},
    {"Cave Corner 01 Lights Type B", subt::ConnectionHelper::CAVE},
    {"Cave Corner 02 Type B", subt::ConnectionHelper::CAVE},
    {"Cave Corner 02 Lights Type B", subt::ConnectionHelper::CAVE},
    {"Cave 3 Way 01 Type B", subt::ConnectionHelper::CAVE},
    {"Cave 3 Way 01 Lights Type B", subt::ConnectionHelper::CAVE},
    {"Cave Elevation Type B", subt::ConnectionHelper::CAVE},
    {"Cave Elevation Lights Type B", subt::ConnectionHelper::CAVE},
    {"Cave Vertical Shaft Type B", subt::ConnectionHelper::CAVE},
    {"Cave Vertical Shaft Lights Type B", subt::ConnectionHelper::CAVE},
    {"Cave Cavern Split 01 Type B", subt::ConnectionHelper::CAVE},
    {"Cave Cavern Split 02 Type B", subt::ConnectionHelper::CAVE},
    {"Cave Corner 30 Type B", subt::ConnectionHelper::CAVE},
    {"Cave Corner 30F Type B", subt::ConnectionHelper::CAVE},
    {"Cave Corner 30 D Type B", subt::ConnectionHelper::CAVE},
    {"Cave Corner 30 D Lights Type B", subt::ConnectionHelper::CAVE},
    {"Cave Corner 30F D Type B", subt::ConnectionHelper::CAVE},
    {"Cave Corner 30F D Lights Type B", subt::ConnectionHelper::CAVE},

    {"Cave 2 Way 01 Type A", subt::ConnectionHelper::CAVE},
    {"Cave 3 Way 01 Type A", subt::ConnectionHelper::CAVE},
    {"Cave 3 Way 02 Type A", subt::ConnectionHelper::CAVE},
    {"Cave 3 Way Elevation 01 Type A", subt::ConnectionHelper::CAVE},
    {"Cave 3 Way Elevation 02 Type A", subt::ConnectionHelper::CAVE},
    {"Cave 3 Way Elevation 03 Type A", subt::ConnectionHelper::CAVE},
    {"Cave 4 Way 01 Type A", subt::ConnectionHelper::CAVE},
    {"Cave Cavern Type A", subt::ConnectionHelper::CAVE},
    {"Cave Corner 01 Type A", subt::ConnectionHelper::CAVE},
    {"Cave Corner 02 Type A", subt::ConnectionHelper::CAVE},
    {"Cave Corner 03 Type A", subt::ConnectionHelper::CAVE},
    {"Cave Corner 03 Type A Lights", subt::ConnectionHelper::CAVE},
    {"Cave Corner 04 Type A", subt::ConnectionHelper::CAVE},
    {"Cave Cap Type A", subt::ConnectionHelper::CAVE},
    {"Cave Elevation 01 Type A", subt::ConnectionHelper::CAVE},
    {"Cave Elevation 02 Type A", subt::ConnectionHelper::CAVE},
    {"Cave Elevation Corner Type A", subt::ConnectionHelper::CAVE},
    {"Cave Elevation Straight Type A", subt::ConnectionHelper::CAVE},
    {"Cave Split Type A", subt::ConnectionHelper::CAVE},
    {"Cave Straight Shift Type A", subt::ConnectionHelper::CAVE},
    {"Cave Straight Type A", subt::ConnectionHelper::CAVE},
    {"Cave U Turn 01 Type A", subt::ConnectionHelper::CAVE},
    {"Cave U Turn Elevation Type A", subt::ConnectionHelper::CAVE},
    {"Cave Vertical Shaft Cantilevered Type A", subt::ConnectionHelper::CAVE},
    {"Cave Vertical Shaft Straight Bottom Type A", subt::ConnectionHelper::CAVE},
    {"Cave Vertical Shaft Straight Top Type A", subt::ConnectionHelper::CAVE},
    {"Cave Vertical Shaft Type A", subt::ConnectionHelper::CAVE},
    {"Cave Vertical Shaft Dead End Type A", subt::ConnectionHelper::CAVE},
    {"Cave 3 Way Elevation 02 Lights Type A", subt::ConnectionHelper::CAVE},
    {"Cave 4 Way 01 Lights Type A", subt::ConnectionHelper::CAVE},
    {"Cave Corner 01 Lights Type A", subt::ConnectionHelper::CAVE},
    {"Cave Corner 02 Lights Type A", subt::ConnectionHelper::CAVE},
    {"Cave Corner 04 Lights Type A", subt::ConnectionHelper::CAVE},
    {"Cave Elevation Straight Lights Type A", subt::ConnectionHelper::CAVE},
    {"Cave Straight Lights Type A", subt::ConnectionHelper::CAVE},
    {"Cave U Turn Elevation Lights Type A", subt::ConnectionHelper::CAVE},
    {"Cave Vertical Shaft Straight Bottom Lights Type A", subt::ConnectionHelper::CAVE},
    {"Cave Transition Type A to and from Type B", subt::ConnectionHelper::CAVE},
    {"Cave Transition Type A to and from Type B Lights", subt::ConnectionHelper::CAVE},

    {"Jenolan Section 01", subt::ConnectionHelper::CAVE},
    {"Jenolan Section 02", subt::ConnectionHelper::CAVE},
    {"Jenolan Section 03", subt::ConnectionHelper::CAVE},
    {"Jenolan Section 04", subt::ConnectionHelper::CAVE},
    {"Jenolan Section 05", subt::ConnectionHelper::CAVE},
    {"Jenolan Section 06", subt::ConnectionHelper::CAVE},
    {"Jenolan Section 07", subt::ConnectionHelper::CAVE},
    {"Jenolan Section 08", subt::ConnectionHelper::CAVE},
    {"Jenolan Section 09", subt::ConnectionHelper::CAVE},
    {"Jenolan Section 10", subt::ConnectionHelper::CAVE},
    {"Jenolan Section 11", subt::ConnectionHelper::CAVE},
    {"Universal Straight 10", subt::ConnectionHelper::UNIVERSAL},
    {"Universal Straight 5", subt::ConnectionHelper::UNIVERSAL},
    {"Universal Straight 2.5", subt::ConnectionHelper::UNIVERSAL},
    {"Universal Shift 5x5", subt::ConnectionHelper::UNIVERSAL},
    {"Universal Shift 2.5x2.5", subt::ConnectionHelper::UNIVERSAL},
    {"Cave Tunnel Transition", subt::ConnectionHelper::TRANSITION},
    {"Cave Tunnel Transition Lights", subt::ConnectionHelper::TRANSITION},
    {"Urban Cave Transition", subt::ConnectionHelper::TRANSITION},
    {"Urban Cave Transition Straight", subt::ConnectionHelper::TRANSITION},
    {"Urban Tunnel Transition", subt::ConnectionHelper::TRANSITION},
    {"Finals Staging Area", subt::ConnectionHelper::STAGING_AREA},
    {"4-Way Finals Transition", subt::ConnectionHelper::TRANSITION},
    {"4-Way Finals Transition 2", subt::ConnectionHelper::TRANSITION},
    {"4-Way Finals Transition 2 Lights", subt::ConnectionHelper::TRANSITION},
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
      auto pt1tf = _tile1->model.RawPose().CoordPositionAdd(pt1);
      ret.push_back(pt1tf);
    }
  }
  return ret;
}
