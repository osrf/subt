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

#include <atomic>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>
#include <ignition/transport/log/Playback.hh>

/// \brief Class that checks certain conditions over an Ignition Transport log
/// file. See Usage() for an example.
class LogChecker
{
  /// \brief Constructor.
  /// \param[in] _argc Number of command line arguments passed to this program.
  /// \param[in] _argv Vector of command line arguments passed to this program.
  public: LogChecker(int _argc,
                     char *_argv[])
  {
    // Parse all command line arguments.
    if (!this->ParseCmdLineArgs(_argc, _argv))
    {
      Usage();
      return;
    }

    // Configure subscribers.
    if (!this->node.Subscribe(this->topic, &LogChecker::PoseCb, this))
    {
      std::cerr << "Error subscribing to topic [" << this->topic
                << "]" << std::endl;
      return;
    }

    // Configure playback.
    this->player.reset(new ignition::transport::log::Playback(this->filePath));
    const int64_t addTopicResult = player->AddTopic(this->topic);
    if (addTopicResult == 0)
    {
      std::cout << "Topic [" << this->topic << "] not found in log file"
                << std::endl;
      return;
    }
    else if (addTopicResult < 0)
    {
      std::cerr << "Failed to advertise topics: [" << addTopicResult
                << "]" << std::endl;
      return;
    }

    this->initialized = true;
  };

  /// \brief Show program usage. This utility checks certain conditions over the
  /// messages stored in an Ignition Transport log file.
  public: void Usage() const
  {
    std::cout << "./log_checker [OPTION]\n\n";
    std::cout << "Required options:\n\n";
    std::cout << "  --file=<FILENAME>  Ignition Transport log file.\n\n";
    std::cout << "  --topic=<TOPIC>    Topic name containing poses.\n\n";
    std::cout << "  --minz=<VALUE>     Minimum Z value allowed.\n\n";
    std::cout << "Example: ./log_checker --file=/tmp/ign/logs/state.tlog ";
    std::cout << "--topic=/world/urban_circuit_practice_01/dynamic_pose/info ";
    std::cout << "--minz=0\n\n";
  }

  /// \brief Parse a single command line argument.
  /// \param[in] _argc Number of command line arguments passed to this program.
  /// \param[in] _argv Vector of command line arguments passed to this program.
  /// \param[in] _option Option to parse. E.g.: --filepath=
  /// \return The value parsed for _option or empty string if not found.
  public: std::string CmdLineArg(int _argc,
                                 char *_argv[],
                                 const std::string &_option) const
  {
    std::string value;
    for (int i = 0; i < _argc; ++i)
    {
      std::string arg = _argv[i];
      size_t index = arg.find(_option);
      if (index == 0)
      {
        value = arg.substr(index + _option.size());
        return value;
      }
    }
    return value;
  }

  /// \brief Parse all command line arguments.
  /// \param[in] _argc Number of command line arguments passed to this program.
  /// \param[in] _argv Vector of command line arguments passed to this program.
  public: bool ParseCmdLineArgs(int _argc,
                                char **_argv)
  {
    // Parse --file
    this->filePath = this->CmdLineArg(_argc, _argv, "--file=");
    if (this->filePath.empty())
      return false;

    // Parse --topic
    this->topic = this->CmdLineArg(_argc, _argv, "--topic=");
    if (this->topic.empty())
      return false;

    // Parse --minz
    if (!this->StringAsDouble(
           this->CmdLineArg(_argc, _argv, "--minz="),
           this->minZ))
    {
      return false;
    }

    return true;
  }

  /// \brief Helper class for converting a string to double.
  /// \param[in] _in Input string.
  /// \param[out] _out Output double.
  public: bool StringAsDouble(const std::string &_in,
                              double &_out) const
  {
    try
    {
      _out = std::stod(_in);
    }
    catch (std::invalid_argument &_e)
    {
      std::cerr << "Unable to convert from string [" << _in << "] to double."
                << std::endl;
      return false;
    }
    catch (std::out_of_range &_e)
    {
      std::cerr << "Unable to convert from string [" << _in << "] to double."
                << "This number is out of range." << std::endl;
      return false;
    }
    return true;
  }

  /// \brief Print the program configuration and results.
  void ShowOutputReport() const
  {
    std::cout << "******************************"                << std::endl;
    std::cout << "Configuration:"                                << std::endl;
    std::cout << "  Log file: ["   << this->filePath << "]"      << std::endl;
    std::cout << "  Topic name: [" << this->topic    << "]"      << std::endl;
    std::cout << "  Minimum Z: ["  << this->minZ     << "]"      << std::endl;
    std::cout << "Output report:"                                << std::endl;
    std::cout << std::boolalpha;
    std::cout << "  Minimum Z check: [" << this->minZPass << "]" << std::endl;
    std::cout << "******************************"                << std::endl;
  }

  /// \brief Ignition Transport callback used to receive all messages to check.
  /// \param[in] _msg A message containing a vector of poses published from the
  /// log file.
  void PoseCb(const ignition::msgs::Pose_V &_msg)
  {
    for (int i = 0; i < _msg.pose_size(); ++i)
    {
      if (_msg.pose(i).position().z() < this->minZ)
        this->minZPass = false;
    }
  }

  /// \brief Verify all checks.
  /// \return 0 when all the checks pass, 1 when the checker isn't properly
  /// initialized, 2 when there's a problem with the playback and 3 when at
  /// least one check doesn't pass.
  int StartAllChecks()
  {
    if (!this->initialized)
      return 1;

    // Begin playback as fast as possible.
    const auto handle = this->player->Start(std::chrono::seconds(1));
    if (!handle)
    {
      std::cerr << "Failed to start playback" << std::endl;
      return 2;
    }

    // Wait until the player stops on its own
    handle->WaitUntilFinished();

    // Print verbose results.
    this->ShowOutputReport();

    return this->minZPass ? 0 : 3;
  }

  /// \brief A transport node.
  ignition::transport::Node node;

  /// \brief The log player.
  std::unique_ptr<ignition::transport::log::Playback> player;

  /// \brief Path to the Ignition Transport log file.
  private: std::string filePath;

  /// \brief Topic name to check.
  private: std::string topic;

  /// \brief Min Z value specified by the user.
  private: double minZ;

  /// \brief Whether this object has been initialized or not.
  private: bool initialized = false;

  /// \brief Check minimum height.
  private: std::atomic<bool> minZPass = true;
};

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  LogChecker logChecker(argc, argv);
  return logChecker.StartAllChecks();
}
