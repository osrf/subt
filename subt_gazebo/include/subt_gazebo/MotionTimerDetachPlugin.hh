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

#ifndef SUBT_GAZEBO_MOTIONTIMERDETACHPLUGIN_HH_
#define SUBT_GAZEBO_MOTIONTIMERDETACHPLUGIN_HH_

#include <memory>
#include <sdf/Element.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <subt_gazebo/JointMotionTimerPlugin.hh>

namespace subt
{
  class MotionTimerDetachPluginPrivate;

  /// \brief A plugin that disables a model by detaching its joints after it
  /// has been in motion for a specified amount of time.
  ///
  /// This plugin is a derived class of JointMotionTimerPlugin, which checks
  /// whether any of the joint velocities in a model exceed
  /// a threshold value. If so, a timer is incremented by the current timestep.
  /// When that timer exceeds a specified amount of time, the model's joints
  /// are detached.
  /// Please see JointMotionTimerPlugin.hh for a full description of its
  /// parameters.
  ///
  /// All non-fixed joints will be checked for motion and detached if the
  /// <all_joints> tag is supplied in the sdf.
  /// Alternatively, individual joint names to check
  /// can be specified in <joint> sdf parameters.
  ///
  /// The <detach_time_limit> sdf parameter can be use to specify the
  /// time in motion (in seconds) after which the robot will be disabled.
  /// The default time limit is 20 minutes.
  ///
  /// Example:
  /// <all_joints/>
  /// <detach_time_limit>1800</detach_time_limit>
  /// ...
  ///
  class MotionTimerDetachPlugin : public subt::JointMotionTimerPlugin
  {
    /// \brief Constructor.
    public: MotionTimerDetachPlugin();

    /// \brief Destructor.
    public: ~MotionTimerDetachPlugin();

    /// \brief Plugin Load function
    /// \param[in] _parent Model pointer to the model defining this plugin
    /// \param[in] _sdf pointer to the SDF of the model
    public: void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Callback for world update events.
    public: void OnUpdate();

    /// \brief Private data pointer.
    private: std::unique_ptr<MotionTimerDetachPluginPrivate> dataPtr;
  };
}
#endif
