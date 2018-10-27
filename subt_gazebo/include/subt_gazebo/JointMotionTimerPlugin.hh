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

#ifndef SUBT_GAZEBO_JOINTMOTIONTIMERPLUGIN_HH_
#define SUBT_GAZEBO_JOINTMOTIONTIMERPLUGIN_HH_

#include <memory>
#include <vector>
#include <ignition/common/Time.hh>
#include <sdf/Element.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Model.hh>

namespace subt
{
  class JointMotionTimerPluginPrivate;

  /// \brief A plugin to track the elapsed time for which a model's joints
  /// have been moving.
  ///
  /// This plugin checks whether any of the joint velocities in a model exceed
  /// a threshold value. If so, a timer is incremented by the current timestep.
  /// The plugin publishes the elapsed time as an ignition::msgs::Duration
  /// on two ignition topics (with optional namespace robotNamespace):
  ///
  /// * /${robotNamespace}/elapsed_time: publishes every simulation step.
  /// * /${robotNamespace}/elapsed_time_updates: publishes only when
  ///   joint motion is detected.
  ///
  /// All non-fixed joints will be checked for motion if the <all_joints> tag
  /// is supplied in the sdf. Alternatively, individual joint names to check
  /// can be specified in <joint> sdf parameters.
  ///
  /// The <joint_velocity_threshold> sdf parameter can be use to specify the
  /// joint velocity threshold above which motion is detected and time elapsed.
  ///
  /// An ignition namespace is used if the <robotNamespace> sdf parameter is
  /// supplied. Otherwise the topics will be published in the global namespace.
  ///
  /// Examples:
  /// <robotNamespace>r2d2</robotNamespace>
  /// <all_joints/>
  /// <joint_velocity_threshold>1e-2</joint_velocity_threshold>
  /// ...
  /// <robotNamespace>c3po</robotNamespace>
  /// <joint>left_elbow</joint>
  /// <joint>right_elbow</joint>
  /// <joint_velocity_threshold>1e-6</joint_velocity_threshold>
  /// ...
  ///
  class JointMotionTimerPlugin : public gazebo::ModelPlugin
  {
    /// \brief Constructor.
    public: JointMotionTimerPlugin();

    /// \brief Plugin Load function
    /// \param[in] _parent Model pointer to the model defining this plugin
    /// \param[in] _sdf pointer to the SDF of the model
    public: void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Get the current amount of time the model has been in motion.
    public: ignition::common::Time ElapsedTime() const;

    /// \brief Get the joints currently checked by this plugin.
    public: std::vector<gazebo::physics::JointPtr> Joints() const;

    /// \brief Callback for world update events.
    public: void OnUpdate();

    /// \brief Private data pointer.
    private: std::unique_ptr<JointMotionTimerPluginPrivate> dataPtr;
  };
}
#endif
