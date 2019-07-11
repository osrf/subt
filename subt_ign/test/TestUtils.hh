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

#ifndef SUBT_GAZEBO_TESTUTILS_HH_
#define SUBT_GAZEBO_TESTUTILS_HH_

#include <std_msgs/Int32.h>
#include <ros/ros.h>
#include <chrono>

namespace subt
{
  /// \brief Utils for writting tests with Gazebo.
  class GazeboTest
  {
    /// \brief Constructor.
    public: GazeboTest()
      : messageReceived(false),
        subscriber(
          this->nodeHandle.subscribe("/subt/score", 5,
                                     &GazeboTest::Callback, this))
    {
    }

    /// \brief Wait until Gazebo is ready.
    /// \param[in] _maxWait Max amount of time to wait. The parameter is the
    /// the same type as the one passed to std::this_thread::sleep_for().
    /// \return True if Gazebo started or false otherwise.
    /// \ref https://en.cppreference.com/w/cpp/thread/sleep_for
    protected: template< class Rep, class Period >
    bool WaitForGazebo(const std::chrono::duration<Rep, Period> &_maxWait)
    {
      auto end = std::chrono::high_resolution_clock::now() + _maxWait;
      while (!this->messageReceived)
      {
        if (std::chrono::high_resolution_clock::now() >= end)
          return false;

        ros::spinOnce();
      }

      return true;
    }

    /// \brief This callback is used to detect when a message from Gazebo is
    /// received.
    /// \param[in] _states Not used.
    private: void Callback(const std_msgs::Int32 &/*_states*/)
    {
      this->messageReceived = true;
    }

    /// \brief Flag to signal when a ROS message is received in a callback.
    private: bool messageReceived;

    /// \brief The ROS node handler.
    private: ros::NodeHandle nodeHandle;

    /// \bried Used to subscribe to a Gazebo ROS topic.
    private: ros::Subscriber subscriber;
  };
}

#endif
