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

#include <subt_ign/Common.hh>
#include <subt_ign/Config.hh>
#include <subt_ign/SimpleDOTParser.hh>

#include <ros/package.h>

using namespace subt;
using namespace ignition;

/////////////////////////////////////////////////
bool ConnectionValidatorPrivate::Load(const std::string &_worldName)
{
  std::string worldsDirectory = ignition::common::joinPaths(
		      ros::package::getPath("subt_ign"), "worlds");

  std::string fullPath;
  subt::FullWorldPath(_worldName, fullPath);
  std::string worldPath = fullPath + ".sdf";
  std::string graphPath = fullPath + ".dot";

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
        dd.tileType = "Cave Starting Area Type B";
      dd.tileName = "staging_area";
    }

    this->vertData[data[4]] = dd;
  }

  // Add SDF::Model data to each of the vertices
  // Assuming 1 world per SDF, which is true for SubT.
  auto world = this->sdfRoot.WorldByIndex(0);

  std::vector<ignition::math::Vector3d> caps;

  for (uint64_t modelIndex = 0; modelIndex < world->ModelCount(); ++modelIndex)
  {
    auto model = world->ModelByIndex(modelIndex);
    const auto pose = model->RawPose();
    const auto name = model->Name();

    if (this->vertData.count(name))
    {
      this->vertData[name].model = *model;
    }
    else if (name.find("cap") != std::string::npos)
    {
      caps.push_back(pose.Pos());
    }
  }

  std::map<std::string, int> expectedConnections;
  std::map<std::string, int> actualConnections;

  for (const auto [name, data]: vertData)
  {
    auto points = ConnectionHelper::connectionPoints[data.tileType];
    expectedConnections[name] = points.size();
    actualConnections[name] = 0;
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
    if(ConnectionHelper::ComputePoint(c.tile1, c.tile2, c.connectionPoint))
    {
      actualConnections[conn1->first]++;
      actualConnections[conn2->first]++;
      connData.push_back(c);
    }
  }

  igndbg << "Populated " << connData.size() << "/" << graph.Edges().size() << std::endl;

  for (const auto [name, data]: expectedConnections)
  {
    auto actual = actualConnections[name];

    if (actual != data)
    {
      // Check against caps
      int found_caps = 0;
      auto points = ConnectionHelper::GetConnectionPoints(&this->vertData[name]);
      for (auto point: points)
      {
        for (auto cap: caps)
        {
          if (point.Equal(cap, 1.0))
          {
            igndbg << "Found cap!" << std::endl;
            found_caps += 1;
          }
          else if (point.Equal(cap, 12.6))
          {
            igndbg << "Found cap (with expanded tolerance)" << std::endl;
            found_caps += 1;
          }
        }
      }

      if (actual + found_caps != data)
      {
        igndbg << "Actual connections[" << actualConnections[name]
               << "] do not match expected connections[" << data
               << "] for tile name[" << name << "] with type["
               << this->vertData[name].tileType << "]. "
               << "Missing an end cap or connecting tile."
               << std::endl;
      }
    }
  }

  this->connDataIter = this->connData.end();
}
