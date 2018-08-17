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

#include <std_msgs/Int32.h>
#include <functional>
#include <utility>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/common/Events.hh>
#include <ignition/math/Pose3.hh>

#include "subt_gazebo/GameLogicPlugin.hh"

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(GameLogicPlugin)

/////////////////////////////////////////////////
void GameLogicPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_world, "GameLogicPlugin world pointer is NULL");
  this->world = _world;

  // Parse the objects of interests.
  this->ParseObjectsOfInterest(_sdf);

  // Initialize the ROS node.
  this->rosnode.reset(new ros::NodeHandle("subt"));

  // Advertise the service to call when an object of interest is recognized.
  this->objectOfInterestSrv =
    this->rosnode->advertiseService("objects_of_interest/new",
    &GameLogicPlugin::OnNewObjectOfInterest, this);

  this->scorePub = this->rosnode->advertise<std_msgs::Int32>("score", 1000);

  // Publish the score.
  this->scoreTimer = this->rosnode->createTimer(
    ros::Duration(1.0), &GameLogicPlugin::PublishScore, this);

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&GameLogicPlugin::OnUpdate, this));

  this->node.Subscribe("/subt/start/contain",
    &GameLogicPlugin::OnStart, this);
  this->node.Subscribe("/subt/finish/contain",
    &GameLogicPlugin::OnFinish, this);

  gzmsg << "Starting SubT" << std::endl;
}

/////////////////////////////////////////////////
void GameLogicPlugin::ParseObjectsOfInterest(sdf::ElementPtr _sdf)
{
  sdf::ElementPtr objectOfInterestElem = _sdf->GetElement("object_of_interest");
  while (objectOfInterestElem)
  {
    // Sanity check: "Name" is required.
    if (!objectOfInterestElem->HasElement("name"))
    {
      gzerr << "[GameLogicPlugin]: Parameter <name> not found. Ignoring this "
            << "object of interest" << std::endl;
      objectOfInterestElem = _sdf->GetNextElement("object_of_interest");
      continue;
    }
    std::string modelName = objectOfInterestElem->Get<std::string>("name");

    // Sanity check: The model should exist.
    physics::ModelPtr modelPtr = this->world->ModelByName(modelName);
    if (!modelPtr)
    {
       gzerr << "[GameLogicPlugin]: Unable to find model with name ["
             << modelName << "]. Ignoring object of interest" << std::endl;
       objectOfInterestElem = _sdf->GetNextElement("object_of_interest");
       continue;
    }

    // Sanity check: The object of interest shouldn't be repeated.
    if (this->objectsOfInterest.find(modelName)
        != this->objectsOfInterest.end())
    {
      gzerr << "[GameLogicPlugin]: Repeated model with name ["
             << modelName << "]. Ignoring object of interest" << std::endl;
      objectOfInterestElem = _sdf->GetNextElement("object_of_interest");
      continue;
    }

    this->objectsOfInterest[modelName] = modelPtr;
    objectOfInterestElem =
      objectOfInterestElem->GetNextElement("object_of_interest");
  }
}

/////////////////////////////////////////////////
bool GameLogicPlugin::OnNewObjectOfInterest(
  subt_msgs::ObjectOfInterest::Request &_req,
  subt_msgs::ObjectOfInterest::Response &/*_res*/)
{
  gzmsg << "New object of interest reported." << std::endl;

  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->totalScore += this->ScoreObjectOfInterest(_req.pose);
    gzmsg << "Total score: " << this->totalScore << std::endl << std::endl;
  }

  return true;
}

double GameLogicPlugin::ScoreObjectOfInterest(
  const geometry_msgs::PoseStamped &_pose)
{
  // Sanity check: Make sure that we still have objects of interest.
  if (this->objectsOfInterest.empty())
  {
    gzmsg << "  No remaining objects of interest" << std::endl;
    return 0.0;
  }

  // Sanity check: Make sure that we have crossed the starting gate.
  if (!this->started)
  {
    gzmsg << "  The task hasn't started yet" << std::endl;
    return 0.0;
  }

  // From the list of all the objects of interest, find out which one is
  // closer (Euclidean distance) to the located by this request.
  ignition::math::Vector3d observedObjectPose(
    _pose.pose.position.x, _pose.pose.position.y, _pose.pose.position.z);
  std::tuple<std::string, ignition::math::Vector3d, double> minDistance =
    {"", ignition::math::Vector3d(), std::numeric_limits<double>::infinity()};
  for (auto const object : this->objectsOfInterest)
  {
    auto objName = object.first;
    auto objPosition = object.second->WorldPose().Pos();
    double distance = observedObjectPose.Distance(objPosition);

    if (distance < std::get<2>(minDistance))
    {
      std::get<0>(minDistance) = objName;
      std::get<1>(minDistance) = objPosition;
      std::get<2>(minDistance) = distance;
    }
  }

  const double kBaseValue = 1.0;
  double score = kBaseValue;

  // Calculate the score based on accuracy in the location.
  double distance = std::get<2>(minDistance);
  if (distance < 0.5)
  {
    gzmsg << "  [Distance bonus]: x3" << std::endl;
    score *= 3;
  }
  else if (distance < 2.0)
  {
    gzmsg << "  [Distance bonus]: x2" << std::endl;
    score *= 2;
  }
  else if (distance < 4.0)
  {
    gzmsg << "  [Distance bonus]: x1" << std::endl;
    score *= 1;
  }
  else
  {
    gzmsg << "  [Distance bonus]: -1" << std::endl;
    score += -1;
  }

  // Apply factors based on the time since the start of the run.
  auto now = std::chrono::steady_clock::now();
  auto elapsedSecs = std::chrono::duration_cast<std::chrono::seconds>(
    now - this->startTime).count();
  if (elapsedSecs < 60 * 20)
  {
    gzmsg << "  [Time bonus]: x3" << std::endl;
    score *= 3.0;
  }
  else if (elapsedSecs < 60 * 40)
  {
    gzmsg << "  [Time bonus]: x2" << std::endl;
    score *= 2.0;
  }
  else
  {
    gzmsg << "  [Time bonus]: x1" << std::endl;
    score *= 1.0;
  }

  gzmsg << "  [Total]: " << score << std::endl;

  // Remove this object of interest to avoid getting score from the same object
  // of interest multiple times.
  this->objectsOfInterest.erase(std::get<0>(minDistance));

  return score;
}

/////////////////////////////////////////////////
void GameLogicPlugin::PublishScore(const ros::TimerEvent &/*_event*/)
{
  std_msgs::Int32 msg;

  {
    std::lock_guard<std::mutex> lock(this->mutex);
    msg.data = this->totalScore;
    this->scorePub.publish(msg);
  }
}

/////////////////////////////////////////////////
void GameLogicPlugin::OnUpdate()
{
}

/////////////////////////////////////////////////
void GameLogicPlugin::OnStart(const ignition::msgs::Boolean &_msg)
{
  if (this->started || !_msg.data())
    return;

  this->started = true;
  this->startTime = std::chrono::steady_clock::now();
  gzmsg << "Scoring has Started" << std::endl;
}

/////////////////////////////////////////////////
void GameLogicPlugin::OnFinish(const ignition::msgs::Boolean &_msg)
{
  if (!this->started || this->finished || !_msg.data())
    return;

  this->finished = true;
  this->finishTime = std::chrono::steady_clock::now();

  auto elapsed = this->finishTime - this->startTime;
  gzmsg << "Scoring has finished. Elapsed time: "
        << std::chrono::duration_cast<std::chrono::seconds>(elapsed).count()
        << " seconds" << std::endl;
}
