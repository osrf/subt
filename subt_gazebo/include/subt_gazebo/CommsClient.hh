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
#ifndef SUBT_GAZEBO_COMMSCLIENT_HH_
#define SUBT_GAZEBO_COMMSCLIENT_HH_

#include <cstdint>
#include <functional>
#include <iostream>
#include <string>

#include <ignition/transport/Node.hh>

#include "subt_gazebo/protobuf/datagram.pb.h"

namespace subt
{
  /// \brief ToDo.
  class CommsClient
  {
    /// \brief Constructor.
    /// \param[in] _localAddress Your local address.
    explicit CommsClient(const std::string &_localAddress);

    /// \brief Get your local address.
    /// \return The local address.
    public: std::string Host() const;

    /// \brief This method can bind a local address and a port to a
    /// virtual socket. This is a required step if your agent needs to
    /// receive messages.
    ///
    /// \param[in] _cb Callback function to be executed when a new message is
    /// received associated to the specified <_address, port>.
    /// In the callback, "_srcAddress" contains the address of the sender of
    /// the message. "_dstAddress" contains the destination address. "_dstPort"
    /// contains the destination port. "_data" contains the payload.
    /// \param[in] _obj Instance containing the member function callback.
    /// \param[in] _address Local address or "kMulticast". If you specify your
    /// local address, you will receive all the messages sent where the
    /// destination is <YOUR_LOCAL_ADDRESS, port> or <"kBroadcast", port>. On
    /// the other hand, if you specify "kMulticast" as the _address parameter,
    /// you will be subscribed to the multicast group <"kMulticast, port>".
    /// You will receive all the messages sent from any node to this multicast
    /// group.
    /// \param[in] _port Port used to receive messages.
    /// \return True when success or false otherwise.
    ///
    /// * Example usage (bind on the local address and default port):
    ///    this->Bind(&MyClass::OnDataReceived, this, "192.168.1.3");
    /// * Example usage (Bind on the multicast group and custom port.):
    ///    this->Bind(&MyClass::OnDataReceived, this, this->kMulticast, 5123);
    public: template<typename C>
    bool Bind(void(C::*_cb)(const std::string &_srcAddress,
                            const std::string &_dstAddress,
                            const uint32_t _dstPort,
                            const std::string &_data),
              C *_obj,
              const std::string &_address = "",
              const int _port = kDefaultPort)
    {
      // Sanity check: Make sure that you use your local address or multicast.
      if ((_address != this->kMulticast) && (_address != this->Host()))
      {
        std::cerr << "[" << this->Host() << "] Bind() error: Address ["
                  << _address << "] is not your local address" << std::endl;
        return false;
      }

      this->cb = std::bind(_cb, _obj, std::placeholders::_1,
        std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);

      // Mapping the "unicast socket" to a topic name.
      const auto endPoint = _address + ":" + std::to_string(_port);

      // Advertise a oneway service for receiving message requests.
      return node.Advertise(endPoint, &CommsClient::OnMessage, this);
    }

    /// \brief Send some data to other/s member/s of the team.
    ///
    /// \param[in] _data Payload.
    /// \param[in] _dstAddress Destination address. Note that the destination
    /// address might be a unicast address, "kBroadcast" or "kMulticast".
    /// In the case of broadcast and multicast communications your node
    /// will receive your own message if you're bind to your local or the
    /// multicast address.
    /// \param[in] _port Destination port.
    /// \return True when success or false if the underlying library used for
    /// sending messages notifies an error (meaning that the message was not
    /// sent).
    public: bool SendTo(const std::string &_data,
                        const std::string &_dstAddress,
                        const uint32_t _port = kDefaultPort);

    /// \brief Function called each time a new datagram message is received.
    private: void OnMessage(const msgs::Datagram &_msg);

    /// \def Callback_t
    /// \brief The callback specified by the user when new data is available.
    /// The callback contains four parameters:
    ///   _srcAddress: The source address of the agent sending the message.
    ///   _dstAddress: The destination address of the message.
    ///   _dstPort: The destination port of the message.
    ///   _data: The payload of the message.
    private: using Callback_t =
      std::function<void(const std::string &_srcAddress,
                         const std::string &_dstAddress,
                         const uint32_t _dstPort,
                         const std::string &_data)>;

    /// \brief The local address.
    private: const std::string localAddress;

    /// \brief Address used to send a message to all the members of the swarm
    /// listening on a specific port.
    private: const std::string kBroadcast = "broadcast";

    /// \brief Address used to bind to a multicast group. Note that we do not
    /// support multiple multicast groups, only one.
    private: const std::string kMulticast = "multicast";

    /// \brief Address used to centralize all messages sent from the agents.
    private: const std::string kBrokerService = "broker";

    /// \brief Default port.
    private: static const uint32_t kDefaultPort = 4100u;

    /// \brief Maximum transmission payload size (octets) for each message.
    private: static const uint32_t kMtu = 1500;

    /// \brief An Ignition Transport node for communications.
    private: ignition::transport::Node node;

    /// \brief The callback registered by the user.
    private: Callback_t cb;
  };
}
#endif
