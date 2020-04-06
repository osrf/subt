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

#include "ConnectionValidatorPrivate.hh"

#include <ignition/common/Console.hh>
#include <ignition/common/Filesystem.hh>
#include <ignition/common/StringUtils.hh>

#include <ignition/fuel_tools/ClientConfig.hh>
#include <ignition/fuel_tools/Interface.hh>

#include <sdf/World.hh>
#include <sdf/Model.hh>

#include <subt_ign/Config.hh>
#include <subt_ign/SimpleDOTParser.hh>

using namespace subt;
using namespace ignition;

/////////////////////////////////////////////////
bool ConnectionValidatorPrivate::Load(const std::string &_worldName)
{
  std::string worldsDirectory = SUBT_INSTALL_WORLD_DIR;

  // Modifications for the tunnel circuit.
  const std::string tunnelPrefix = "tunnel_circuit_";
  const std::string urbanPrefix = "urban_circuit_";
  const std::string cavePrefix = "cave_circuit_";
  if (0 == _worldName.compare(0, tunnelPrefix.size(), tunnelPrefix))
  {
    std::string suffix = _worldName.substr(tunnelPrefix.size());
    // don't use a subfolder for practice worlds
    if (0 != suffix.compare(0, 9, "practice_"))
    {
      worldsDirectory = ignition::common::joinPaths(worldsDirectory,
          "tunnel_circuit", suffix);
    }
  }
  else if (0 == _worldName.compare(0, urbanPrefix.size(), urbanPrefix))
  {
    std::string suffix = _worldName.substr(urbanPrefix.size());
    // don't use a subfolder for practice worlds
    if (0 != suffix.compare(0, 9, "practice_"))
    {
      worldsDirectory = ignition::common::joinPaths(worldsDirectory,
          "urban_circuit", suffix);
    }
  }
  else if (0 == _worldName.compare(0, cavePrefix.size(), cavePrefix))
  {
    std::string suffix = _worldName.substr(cavePrefix.size());
    // don't use a subfolder for practice worlds
    if (0 != suffix.compare(0, 9, "practice_"))
    {
      worldsDirectory = ignition::common::joinPaths(worldsDirectory,
          "cave_circuit", suffix);
    }
  }
  else if (_worldName.find("simple") == std::string::npos)
  {
    ignwarn << "Unable to determine circuit number from["
      << _worldName << "].\n";
  }

  std::string worldPath = common::joinPaths(
      worldsDirectory, _worldName + ".sdf");

  std::string graphPath = common::joinPaths(
      worldsDirectory, _worldName + ".dot");

  auto ret = LoadSdf(worldPath) && LoadDot(graphPath);
  if (ret)
    this->worldName = _worldName;
  return ret;
}

/////////////////////////////////////////////////
bool ConnectionValidatorPrivate::LoadSdf(const std::string &_fpath)
{
  // Configure the fuel client
  fuel_tools::ClientConfig config;
  fuel_tools::FuelClient fuelClient(config);

  sdf::setFindCallback([&](const std::string &_uri) {
      return fuel_tools::fetchResourceWithClient(_uri, fuelClient);
  });

  igndbg << "Parsing [" << _fpath << "]" << std::endl;

  auto errors = sdfRoot.Load(_fpath);
  if (!errors.empty())
  {
    for (auto &err: errors)
      ignerr << err << "\n";
    return false;
  }
  return true;
}

/////////////////////////////////////////////////
bool ConnectionValidatorPrivate::LoadDot(const std::string &_fpath)
{
  std::filebuf fb;
  if (!fb.open(_fpath, std::ios::in))
  {
    ignerr << "Unable to read [" << _fpath << "] file" << std::endl;
    return false;
  }

  igndbg << "Parsing [" << _fpath << "]" << std::endl;

  std::istream is(&fb);
  subt::SimpleDOTParser dotParser;
  if (!dotParser.Parse(is))
    return false;

  this->graph = dotParser.Graph();

  return true;
}

/////////////////////////////////////////////////
bool ConnectionValidatorPrivate::Move(const ignition::math::Vector3d& pt)
{
  ignition::msgs::Pose req;
  req.set_name("validator");
  req.mutable_position()->set_x(pt.X());
  req.mutable_position()->set_y(pt.Y());
  req.mutable_position()->set_z(pt.Z());

  msgs::Boolean res;
  bool result;
  unsigned int timeout = 1000;
  std::string service {"/world/" + this->worldName + "/set_pose"};
  node.Request(service, req, timeout, res, result);
  igndbg << "Result: " << res.data() << " " << result << std::endl;
  return result;
}

/////////////////////////////////////////////////
bool ConnectionValidatorPrivate::OnNext(const ignition::msgs::StringMsg& /*_req*/,
                                      ignition::msgs::StringMsg& _rep)
{
  this->connDataIter++;

  if (this->connDataIter >= this->connData.end())
  {
    this->connDataIter = this->connData.begin();
  }

  std::stringstream ss;
  ss << "Moving to: "
    << "[(" << this->connDataIter->tile1->id << ") "
    << this->connDataIter->tile1->tileName << "(" << this->connDataIter->tile1->tileType << ")]"
    << " -- "
    << "[(" << this->connDataIter->tile2->id << ") "
    << this->connDataIter->tile2->tileName << "(" << this->connDataIter->tile2->tileType << ")]\n"
    << "[" << this->connDataIter->connectionPoint << "]" << std::endl;

  _rep.set_data(ss.str());

  igndbg << ss.str() << std::endl;
  return this->Move(this->connDataIter->connectionPoint);
}

/////////////////////////////////////////////////
bool ConnectionValidatorPrivate::OnPrev(const ignition::msgs::StringMsg& /*_req*/,
                                      ignition::msgs::StringMsg& _rep)
{
  this->connDataIter--;

  std::stringstream ss;
  ss << "Moving to: "
    << "[(" << this->connDataIter->tile1->id << ") "
    << this->connDataIter->tile1->tileName << "(" << this->connDataIter->tile1->tileType << ")]"
    << " -- "
    << "[(" << this->connDataIter->tile2->id << ") "
    << this->connDataIter->tile2->tileName << "(" << this->connDataIter->tile2->tileType << ")]\n"
    << "[" << this->connDataIter->connectionPoint << "]" << std::endl;

  _rep.set_data(ss.str());

  igndbg << ss.str() << std::endl;
  auto ret = this->Move(this->connDataIter->connectionPoint);

  if (this->connDataIter == this->connData.begin())
  {
    this->connDataIter = this->connData.end();
    this->connDataIter--;
  }
  return ret;
}

/////////////////////////////////////////////////
void ConnectionValidatorPrivate::PopulateConnections()
{
  // Pull vertex data from the DOT graph file.
  auto verts = this->graph.Vertices();
  for (const auto vert : verts)
  {
    auto vv = vert.second.get();
    auto data = ignition::common::Split(
        vv.Data().substr(1, vv.Data().length() - 2), ':');
    VertexData dd;

    dd.id = stoi(data[0]);
    dd.tileType = data[2];
    dd.tileName = data[4];

    if (dd.tileType == "base_station")
    {
      if (this->worldName.find("urban") != std::string::npos)
        dd.tileType = "Urban Starting Area";
      else if (this->worldName.find("cave") != std::string::npos)
        dd.tileType = "Cave Starting Area";
      dd.tileName = "staging_area";
    }

    this->vertData[data[4]] = dd;
  }

  // Add SDF::Model data to each of the vertices
  // Assuming 1 world per SDF, which is true for SubT.
  auto world = this->sdfRoot.WorldByIndex(0);

  for (uint64_t modelIndex = 0; modelIndex < world->ModelCount(); ++modelIndex)
  {
    auto model = world->ModelByIndex(modelIndex);
    const auto pose = model->Pose();
    const auto name = model->Name();

    if (this->vertData.count(name))
    {
      this->vertData[name].model = *model;
    }
  }

  // Pull edge data from the DOT graph file.
  auto edges = this->graph.Edges();

  for (const auto [id, edge]: edges)
  {
    auto conn1 = std::find_if(this->vertData.begin(),
        this->vertData.end(),
        [&]( const auto & vert ) {
          return vert.second.id == static_cast<int>(edge.get().Vertices().first);
        });

    auto conn2 = std::find_if(this->vertData.begin(),
        this->vertData.end(),
        [&]( const auto & vert ) {
          return vert.second.id == static_cast<int>(edge.get().Vertices().second);
        });

    if (conn1 == this->vertData.end())
    {
      ignerr << "Could not find vertex information for ID: "
        << edge.get().Vertices().first << std::endl;
      continue;
    }

    if (conn2 == this->vertData.end())
    {
      ignerr << "Could not find vertex information for ID: "
        << edge.get().Vertices().second << std::endl;
      continue;
    }

    Connection c;
    c.id = id;
    c.tile1 = &conn1->second;
    c.tile2 = &conn2->second;
    // Only add to the list if a connection point can be computed.
    if(ComputePoint(c.tile1, c.tile2, c.connectionPoint))
    {
      connData.push_back(c);
    }
  }

  igndbg << "Populated " << connData.size() << "/" << graph.Edges().size() << std::endl;

  this->connDataIter = this->connData.end();
}

/////////////////////////////////////////////////
bool subt::ComputePoint(VertexData *_tile1, VertexData *_tile2,
    ignition::math::Vector3d& _pt)
{
  std::map<std::string,
        std::vector<ignition::math::Vector3d>> connectionPoints = {
    {"Urban Straight", {{0, 20, 0}, {0, -20, 0}}},
    {"Urban Bend Right", {{0, -20, 0}, {20, 0, 0}}},
    {"Urban Bend Left", {{0, -20, 0}, {-20, 0, 0}}},
    {"Urban Superpose", {{0, 20, 0}, {0, -20, 0}, {-20, 0, 10}, {20, 0, 10}}},
    {"Urban 3-Way Right Intersection", {{0, 20, 0}, {0, -20, 0}, {20, 0, 0}}},
    {"Urban Straight Door Right", {{20, 0, 0}, {-20, 0, 0}, {-16.021, 3.94, 0.94}}},
    {"Urban Straight Door Left", {{20, 0, 0}, {-20, 0, 0}, {-16.021, -3.94, 0.94}}},
    {"Urban Straight Door Right Flipped", {{20, 0, 0}, {-20, 0, 0}, {-16.021, 3.94, 0.94}}},
    {"Urban Straight Door Left Flipped", {{20, 0, 0}, {-20, 0, 0}, {-16.021, -3.94, 0.94}}},
    {"Urban Straight Door Right Extension", {{20, 0, 0}, {-20, 0, 0}, {0, 20, 0}}},
    {"Urban Service Room Centered", {{0, 20, 0}}},
    {"Urban Service Room", {{-16.023, 3.906, 0.919}}},
    {"Urban Service Room Straight", {{0, -20, 0}, {0, 20, 0}}},
    {"Urban Platform", {{20, 0, 0}, {-20, 0, 0}, {0, 20, 1.7}, {23.979, 3.906, 0.94}, {-23.979, 3.906, 0.94}}},
    {"Urban Platform Open", {{20, 0, 0}, {-20, 0, 0}, {0, 20, 0},
      {23.979, 3.906, 0.919}, {-23.979, 3.906, 0.919}, {23.982, 11.743, 0.919}}},
    {"Urban Stairwell Platform", {{0, 20, 11.69}, {0, -20, 1.69}}},
    {"Urban Stairwell Platform Centered", {{0, 20, 10}, {0, -20, 0}}},
    {"Urban Starting Area", {{-16.021, 3.94, 0.919}}},
    {"Urban Elevation Up", {{0, 20, 5}, {0, -20, 0}}},
    {"Urban Elevation Down", {{0, 20, 0}, {0, -20, 5}}},
    {"Urban 2 Story", {{0, 20, 10}, {0, -20, 0}, {-20, 0, 10}, {20, 0, 0}}},
    {"Urban 2 Story Large Side 1", {{0, 20, 10}, {0, -20, 0}}},
    {"Urban 2 Story Large Side 2", {{0, -20, 0}, {0, 20, 0}}},
    {"Urban Large Room Split", {{0, -20, 0}, {-20, 0, 0}, {0, 20, 0}}},
    {"Cave Starting Area", {{12.5, 0, 0}}},
    {"Cave Straight 01", {{0, 12.5, 0}, {0, -12.5, 0}}},
    {"Cave Straight 02", {{0, 12.5, 0}, {0, -12.5, 0}}},
    {"Cave Straight 03", {{0, 12.5, 0}, {0, -12.5, 0}}},
    {"Cave Straight 04", {{0, 12.5, 0}, {0, -12.5, 0}}},
    {"Cave Straight 05", {{0, 12.5, 0}, {0, -12.5, 0}}},
    {"Cave Corner 01", {{-12.5, 0, 0}, {0, 12.5, 0}}},
    {"Cave Corner 02", {{-12.5, 0, 0}, {0, 12.5, 0}}},
    {"Cave 3 Way 01", {{12.5, 0, 0}, {-12.5, 0, 0}, {0, 12.5, 0}}},
    {"Cave Elevation", {{0, 12.5, 0}, {0, -12.5, 10}}},
    {"Cave Vertical Shaft", {{0, 12.5, 20}, {0, -12.5, 0}}},
    {"Cave Cavern Split 01", {{0, 12.5, 12.5}, {12.5, 0, 0}, {-12.5, 0, 0}}},
    {"Cave Cavern Split 02", {{12.5, 0, 0}, {-12.5, 0, 0}}},
  };

  std::vector<std::string> lights = {
    "Urban Straight",
    "Urban Bend Left",
    "Urban Straight Door Right Extension",
    "Urban Service Room Centered",
    "Urban Service Room",
    "Urban Service Room Straight",
    "Urban Stairwell Platform",
    "Urban Stairwell Platform Centered",
    "Urban Elevation Up",
    "Urban 2 Story",
    "Urban 2 Story Large Side 1",
    "Urban 2 Story Large Side 2",
    "Urban Large Room Split",
    "Urban Straight Door Right Flipped"
  };

  for (auto ll : lights)
  {
    connectionPoints[ll + " Lights"] = connectionPoints[ll];
  }

  if (!connectionPoints.count(_tile1->tileType))
  {
    ignwarn << "No connection information for: " << _tile1->tileType << std::endl;
    return false;
  }

  if (!connectionPoints.count(_tile2->tileType))
  {
    ignwarn << "No connection information for: " << _tile2->tileType << std::endl;
    return false;
  }

  for (const auto& pt1 : connectionPoints[_tile1->tileType])
  {
    auto pt1tf = _tile1->model.Pose().CoordPositionAdd(pt1);
    for (const auto& pt2 : connectionPoints[_tile2->tileType])
    {
      auto pt2tf = _tile2->model.Pose().CoordPositionAdd(pt2);
      if (pt1tf.Equal(pt2tf, 1))
      {
        _pt = pt1tf;
        return true;
      }
    }
  }

  ignwarn << "Failed to connect: " << _tile1->tileType << " " << _tile2->tileType << std::endl;

  for (const auto& pt1 : connectionPoints[_tile1->tileType])
  {
    auto pt1tf = _tile1->model.Pose().CoordPositionAdd(pt1);
    for (const auto& pt2 : connectionPoints[_tile2->tileType])
    {
      auto pt2tf = _tile2->model.Pose().CoordPositionAdd(pt2);
      igndbg <<
        _tile1->tileType << " [" << _tile1->model.Pose() << "] -- " <<
        _tile2->tileType << " [" << _tile2->model.Pose() << "] [" << pt1tf << "] [" << pt2tf << "]" << std::endl;
    }
  }

  return false;
}
