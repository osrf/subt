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
#ifndef SUBT_IGN_ROBOTPLATFORMTYPES_HH_
#define SUBT_IGN_ROBOTPLATFORMTYPES_HH_
#include <map>
/// \brief List of robot platform types and kinetic energy threshold factor. This is used to count unique robot types and determine crashes.
const std::map<std::string, double> robotPlatformTypes = {
  {"ABSOLEM", 1},
  {"ALLIE", 1},
  {"ANYMAL_B", 1},
  {"ANYMAL_C", 1},
  {"CRYSTAL", 1},
  {"DS1", 1},
  {"DTR", 1},
  {"FREYJA", 1},
  {"GAGARIN", 1},
  {"HD2", 1},
  {"HOVERMAP", 1},
  {"HUSKY", 1},
  {"KAREN", 1},
  {"KLOUBAK", 1},
  {"LILY", 1},
  {"M100", 1},
  {"MARV", 1},
  {"MIKE", 1},
  {"OZBOT_ATR", 1},
  {"PAM", 1},
  {"QAV500", 1},
  {"R2", 1},
  {"R3", 1},
  {"RMF", 1},
  {"ROCKY", 1},
  {"SHAFTER", 1},
  {"SPOT", 1},
  {"TEAMBASE", 1},
  {"X1", 1},
  {"X2", 1},
  {"X3", 7},
  {"X4", 7.2},
  {"X500", 1},
};

#endif
