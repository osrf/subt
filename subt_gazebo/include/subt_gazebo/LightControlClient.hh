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
#ifndef SUBT_GAZEBO_LIGHTCONTROLCLIENT_HH_
#define SUBT_GAZEBO_LIGHTCONTROLCLIENT_HH_

#include <string>
#include <ignition/transport/Node.hh>

namespace subt
{
  /// \brief ToDo.
  class LightControlClient
  {
    /// \brief Constructor.
    /// \param[in] _light_name Name of the light.
    /// \param[in] _link_name Name of the link which the light is attached on.
    public: explicit LightControlClient(
      const std::string &_light_name, const std::string &_link_name);

    /// \brief Turn on the light.
    public: bool TurnOn();

    /// \brief Turn off the light.
    public: bool TurnOff();

    /// \brief Change the duration.
    public: bool ChangeDuration(const double &_duration);

    /// \brief Change the interval.
    public: bool ChangeInterval(const double &_interval);

    /// \brief The name of the light.
    private: std::string light_name;

    /// \brief The name of the link.
    private: std::string link_name;

    /// \brief An Ignition Transport node for communications.
    private: ignition::transport::Node node;
  };
}
#endif
