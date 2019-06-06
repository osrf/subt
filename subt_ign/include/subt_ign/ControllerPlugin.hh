/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#ifndef SUBT_IGN_CONTROLLER_HH_
#define SUBT_IGN_CONTROLLER_HH_

#include <memory>
#include <string>
#include <sdf/Element.hh>

#include <ignition/launch/Plugin.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <subt_communication_broker/subt_communication_client.h>

namespace subt
{
  /// \brief A timer class to execute a function periodically.
  /// \ref https://stackoverflow.com/questions/30425772/c-11-calling-a-c-function-periodically
  class CallBackTimer
  {
    /// \brief Class constructor.
    public: CallBackTimer()
      : execute(false)
    {
    }

    /// \brief Class destructor.
    public: ~CallBackTimer()
    {
      if (this->execute.load(std::memory_order_acquire))
        Stop();
    };

    /// \brief Stop the timer.
    public: void Stop()
    {
      this->execute.store(false, std::memory_order_release);
      if (this->thd.joinable())
        this->thd.join();
    }

    /// \brief Start the timer.
    /// \param[in] _interval Interval between function calls (milliseconds).
    /// \param[in] _func Function to execute periodically.
    public: void Start(int _interval, std::function<void(void)> _func)
    {
      if (this->execute.load(std::memory_order_acquire))
        Stop();

      this->execute.store(true, std::memory_order_release);
      this->thd = std::thread([this, _interval, _func]()
      {
        while (this->execute.load(std::memory_order_acquire))
        {
          _func();
          std::this_thread::sleep_for(
          std::chrono::milliseconds(_interval));
        }
      });
    }

    /// \brief Is the the timer running?
    /// \return True if the timer is running or false otherwise.
    public: bool IsRunning() const noexcept
    {
      return this->execute.load(std::memory_order_acquire) &&
             this->thd.joinable();
    }

    /// \brief Condition to execute or not.
    private: std::atomic<bool> execute;

    /// \brief The thread for running the function.
    private: std::thread thd;
  };

  /// \brief A plugin to receive artifact reports from the teams.
  class ControllerPlugin : public ignition::launch::Plugin
  {
    /// \brief Constructor
    public: ControllerPlugin();

    // Documentation inherited
    public: virtual bool Load(const tinyxml2::XMLElement *_elem) override final;

    /// \brief Callback for processing an artifact report.
    /// \param[in] _srcAddress Unused.
    /// \param[in] _dstAddress Unused.
    /// \param[in] _dstPort Unused.
    /// \param[in] _data Serialized artifact.
    private: void OnMsg(const std::string &_srcAddress,
                        const std::string &_dstAddress,
                        const uint32_t _dstPort,
                        const std::string &_data);

    /// \brief SubT communication client.
    private: std::unique_ptr<subt::CommsClient> client;

    /// \brief The robot name.
    private: std::string name;

    /// \brief A callback timer.
    private: CallBackTimer timer;
  };
}

// Register the plugin
IGNITION_ADD_PLUGIN(subt::ControllerPlugin, ignition::launch::Plugin)
#endif
