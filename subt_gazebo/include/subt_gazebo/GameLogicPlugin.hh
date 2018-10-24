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
#ifndef SUBT_GAZEBO_GAMELOGICPLUGIN_HH_
#define SUBT_GAZEBO_GAMELOGICPLUGIN_HH_

#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <array>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <gazebo/common/Event.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/Node.hh>
#include <sdf/sdf.hh>

#include "subt_gazebo/CommonTypes.hh"

// Forward declarations.
namespace subt
{
  namespace msgs
  {
    class Artifact;
  }
}
namespace ignition
{
  namespace msgs
  {
    class Pose;
  }
}

namespace gazebo
{
  /// \brief A plugin that takes care of all the SubT challenge logic.
  class GameLogicPlugin : public WorldPlugin
  {
    /// \brief Mapping between enum types and strings.
    const std::array<
      const std::pair<subt::ArtifactType, std::string>, 8> kArtifactTypes
      {
        {
          {subt::ArtifactType::TYPE_BACKPACK      , "TYPE_BACKPACK"},
          {subt::ArtifactType::TYPE_DUCT          , "TYPE_DUCT"},
          {subt::ArtifactType::TYPE_ELECTRICAL_BOX, "TYPE_ELECTRICAL_BOX"},
          {subt::ArtifactType::TYPE_EXTINGUISHER  , "TYPE_EXTINGUISHER"},
          {subt::ArtifactType::TYPE_PHONE         , "TYPE_PHONE"},
          {subt::ArtifactType::TYPE_RADIO         , "TYPE_RADIO"},
          {subt::ArtifactType::TYPE_TOOLBOX       , "TYPE_TOOLBOX"},
          {subt::ArtifactType::TYPE_VALVE         , "TYPE_VALVE"}
        }
      };

    // Documentation inherited
    public: virtual void Load(physics::WorldPtr _world,
                              sdf::ElementPtr _sdf);

    /// \brief Callback for World Update events.
    private: void OnUpdate();

    /// \brief Callback triggered when a pair of links collide. It starts the
    /// timer if a specified start area is collided by some object.
    /// \param[in] _msg The message containing a list of collision information.
    private: void OnStartCollision(ConstIntPtr &_msg);

    /// \brief ROS service callback triggered when the service is called.
    /// \param[in]  _req The message containing a flag telling if the game is to
    /// be finished.
    /// \param[out] _res The response message.
    private: bool OnFinishCall(std_srvs::SetBool::Request &_req,
                               std_srvs::SetBool::Response &_res);

    /// \brief Parse all the artifacts.
    /// \param[in] _sdf The SDF element containing the artifacts.
    private: void ParseArtifacts(sdf::ElementPtr _sdf);

    /// \brief Callback executed to process a new artifact request
    /// sent by a team.
    /// \param[in] _req The service request.
    private: void OnNewArtifact(const subt::msgs::Artifact &_req);

    /// \brief Calculate the score of a new artifact request.
    /// \param[in] _type The object type. See ArtifactType.
    /// \param[in] _pose The object pose.
    /// \return The score obtained for this object.
    private: double ScoreArtifact(const subt::ArtifactType &_type,
                                  const ignition::msgs::Pose &_pose);

    /// \brief Publish the score.
    /// \param[in] _event Unused.
    private: void PublishScore(const ros::TimerEvent &_event);

    /// \brief Create an ArtifactType from a string.
    /// \param[in] _name The artifact in string format.
    /// \param[out] _type The artifact type.
    /// \return True when the conversion succeed or false otherwise.
    private: bool ArtifactFromString(const std::string &_name,
                                     subt::ArtifactType &_type);

    /// \brief Create an ArtifactType from an integer.
    /// \param[in] _typeInt The artifact in int format.
    /// \param[out] _type The artifact type.
    /// \return True when the conversion succeed or false otherwise.
    private: bool ArtifactFromInt(const uint32_t &_typeInt,
                                  subt::ArtifactType &_type);

    /// \brief World pointer.
    private: physics::WorldPtr world;

    /// \brief Connection to World Update events.
    private: event::ConnectionPtr updateConnection;

    /// \brief Ignition Transport node.
    private: ignition::transport::Node node;

    /// \brief Gazebo Transport node.
    private: gazebo::transport::NodePtr gzNode;

    /// \brief ROS service server to receive a call to finish the game.
    private: ros::ServiceServer finishService;

    /// \brief Gazebo Transport Subscriber to check the collision.
    private: gazebo::transport::SubscriberPtr startCollisionSub;

    /// \brief Whether the task has started.
    private: bool started = false;

    /// \brief Whether the task has finished.
    private: bool finished = false;

    /// \brief Start time used for scoring.
    private: std::chrono::steady_clock::time_point startTime;

    /// \brief Finish time used for scoring.
    private: std::chrono::steady_clock::time_point finishTime;

    /// \brief Store all artifacts.
    /// The key is the object type. See ArtifactType for all supported values.
    /// The value is another map, where the key is the model name and the value
    /// is a Model pointer.
    private: std::map<subt::ArtifactType,
                      std::map<std::string, physics::ModelPtr>> artifacts;

    /// \brief The ROS node handler used for communications.
    private: std::unique_ptr<ros::NodeHandle> rosnode;

    /// \brief Publish the score.
    private: ros::Publisher scorePub;

    /// \brief Total score.
    private: double totalScore = 0.0;

    /// \brief A timer to publish the score at a given frequency.
    private: ros::Timer scoreTimer;

    /// \brief A mutex.
    private: std::mutex mutex;
  };
}
#endif
