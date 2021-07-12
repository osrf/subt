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
#include <ros/ros.h>
#include <cstdint>
#include <functional>
#include <iostream>
#include <map>
#include <mutex>
#include <string>
#include <vector>
#include <ignition/transport/Node.hh>
#include <subt_msgs/DatagramRos.h>

#include <subt_communication_broker/common_types.h>
#include <subt_communication_broker/protobuf/datagram.pb.h>
#include <subt_communication_broker/protobuf/neighbor_m.pb.h>

namespace subt
{

  /// \brief ToDo.
  class CommsClient
  {
    /// \brief Constructor.
    /// \param[in] _localAddress Your local address.
    /// Important: This address must be equal to a Gazebo model name.
    /// \param[in] _isPrivate If true, only nodes within the same process will
    /// be able to communicate with this client.
    /// \param[in] _useIgnition Set to true if you are using Ignition
    /// transport (i.e. not ROS). This is needed by the base station,
    /// and tests. If you are a regular robot, then you really really do not
    /// want to set this to true as your Commsclient will not work. For each
    /// address (robot), there can be only one client with _useIgnition set to
    /// true. There can be multiple clients with _useIgnition set to false.
    /// \param[in] _listenBeacons If true (the default), the commsclient will
    /// listen to beacon packets on port 4000. This (or another regular stream
    /// of data) is required for the Neighbors() function to work.
    /// \param[in] _rosNh The ROS node handle used to create ROS subscribers for
    /// incoming messages. Only used when _useIgnition is false. If not given,
    /// a default node handle is used (in the current namespace).
    public:
    explicit
    CommsClient(const std::string& _localAddress,
                bool _isPrivate = false,
                bool _useIgnition = false,
                bool _listenBeacons = true,
                ros::NodeHandle* _rosNh = nullptr);

    /// \brief Destructor.
    public: virtual ~CommsClient();

    /// \brief Get your local address.
    /// \return The local address.
    public: std::string Host() const;

    /// \brief This method can bind a local address and a port to a
    /// virtual socket. This is a required step if your agent needs to
    /// receive messages. It is possible to bind multiple callbacks to the
    /// same address:port endpoint, even from the same client ID. Just call
    /// Bind() multiple times.
    ///
    /// \param[in] _cb Callback function to be executed when a new message is
    /// received associated to the specified <_address, port>.
    /// In the callback, "_srcAddress" contains the address of the sender of
    /// the message. "_dstAddress" contains the destination address. "_dstPort"
    /// contains the destination port. "_data" contains the payload.
    /// \param[in] _address Local address or "kMulticast". If you specify your
    /// local address, you will receive all the messages sent where the
    /// destination is <YOUR_LOCAL_ADDRESS, port> or <"kBroadcast", port>. On
    /// the other hand, if you specify "kMulticast" as the _address parameter,
    /// you will be subscribed to the multicast group <"kMulticast, port>".
    /// You will receive all the messages sent from any node to this multicast
    /// group.
    /// \param[in] _port Port used to receive messages.
    /// \return List of endpoints that were created in result of this bind call.
    /// It can be up to two endpoints - one for the unicast/multicast address,
    /// and one for the broadcast address. The returned pairs contain the IDs
    /// of the endpoints and their names (i.e. address:port). If the returned
    /// list is empty, binding failed. You may test for this case using the
    /// overloaded operator! on the result vector.
    ///
    /// * Example usage (bind on the local address and default port):
    ///    this->Bind(&OnDataReceived, "192.168.1.3");
    /// * Example usage (Bind on the multicast group and custom port.):
    ///    this->Bind(&OnDataReceived, this->kMulticast, 5123);
    public:
    std::vector<std::pair<communication_broker::EndpointID, std::string>>
    Bind(std::function<void(const std::string& _srcAddress,
                            const std::string& _dstAddress,
                            uint32_t _dstPort,
                            const std::string& _data)> _cb,
         const std::string& _address = "",
         int _port = communication_broker::kDefaultPort);

    /// \brief This method can bind a local address and a port to a
    /// virtual socket. This is a required step if your agent needs to
    /// receive messages. It is possible to bind multiple callbacks to the
    /// same address:port endpoint, even from the same client ID. Just call
    /// Bind() multiple times.
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
    /// \return List of endpoints that were created in result of this bind call.
    /// It can be up to two endpoints - one for the unicast/multicast address,
    /// and one for the broadcast address. The returned pairs contain the IDs
    /// of the endpoints and their names (i.e. address:port). If the returned
    /// list is empty, binding failed. You may test for this case using the
    /// overloaded operator! on the result vector.
    ///
    /// * Example usage (bind on the local address and default port):
    ///    this->Bind(&MyClass::OnDataReceived, this, "192.168.1.3");
    /// * Example usage (Bind on the multicast group and custom port.):
    ///    this->Bind(&MyClass::OnDataReceived, this, this->kMulticast, 5123);
    public:
    template<typename C>
    std::vector<std::pair<communication_broker::EndpointID, std::string>>
    Bind(void(C::*_cb)(const std::string& _srcAddress,
                       const std::string& _dstAddress,
                       uint32_t _dstPort,
                       const std::string& _data),
         C* _obj,
         const std::string& _address = "",
         int _port = communication_broker::kDefaultPort)
    {
      return this->Bind(std::bind(_cb, _obj,
                                  std::placeholders::_1,
                                  std::placeholders::_2,
                                  std::placeholders::_3,
                                  std::placeholders::_4),
                        _address, _port);
    }

    /// \brief This method can unbind from a "socket" acquired by Bind(). Once
    /// unbound, the registered callback will no longer be called.
    ///
    /// \param[in] _endpointId ID of the endpoint to unbind. This is the ID
    /// returned from Bind() call.
    /// \return Success of the unbinding.
    public: bool Unbind(communication_broker::EndpointID _endpointId);

    /// \brief Send some data to other/s member/s of the team.
    ///
    /// \param[in] _data Payload. The maximum size of the payload is 1500 bytes.
    /// \param[in] _dstAddress Destination address. Note that the destination
    /// address might be a unicast address, "kBroadcast" or "kMulticast".
    /// In the case of broadcast and multicast communications your node
    /// will receive your own message if you're bind to your local or the
    /// multicast address.
    /// \param[in] _port Destination port.
    /// \return True when success or false otherwise (e.g.: if the payload was
    /// bigger than 1500 bytes).
    public: bool SendTo(const std::string &_data,
                        const std::string &_dstAddress,
                        uint32_t _port = communication_broker::kDefaultPort);

    /// \brief Type for storing neighbor data
    public: typedef std::map<std::string, std::pair<double, double>> Neighbor_M;

    /// \brief Get the list of local neighbors.
    ///
    /// \return A map of addresses and signal strength from your
    /// local neighbors.
    /// \note For this function to work reliably, you need to be sure there is
    /// a regular flow of data from all other robots towards this address.
    /// One way to achieve it is to send and listen to beacons (see constructor
    /// argument _listenBeacons and function StartBeaconInterval() or
    /// SendBeacons()).
    public: Neighbor_M Neighbors() const;

    /// \brief Broadcast a BEACON packet
    public: bool SendBeacon();

    /// \brief Start sending beacon packets at the specified interval
    /// \bried Period at which to send the beacon.
    public: void StartBeaconInterval(ros::Duration _period);

    /// \brief Register the current address. This will make a synchronous call
    /// to the broker to validate and register the address.
    /// \return True when the address is valid or false otherwise.
    private: bool Register();

    /// \brief Unregister the current address. This will make a synchronous call
    /// to the broker to unregister the address. It will also unbind all
    /// bound callbacks from this client.
    /// \return True when the unregistration succeeded or false otherwise.
    private: bool Unregister();

    /// \brief Function called each time a new datagram message is received.
    /// \param[in] _msg The incoming message.
    private: void OnMessage(const msgs::Datagram &_msg);

    /// \brief Function called each time a new datagram message is received.
    /// \param[in] _req The incoming message.
    private: void OnMessageRos(const subt_msgs::DatagramRos::Request &_req);

    /// \brief On clock message. This is used primarily/only by the
    /// BaseStation.
    private: void OnClock(const ignition::msgs::Clock &_clock);

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
                         uint32_t _dstPort,
                         const std::string &_data)>;

    /// \brief The local address.
    private: const std::string localAddress;

    /// \brief Maximum transmission payload size (octets) for each message.
    private: static const uint32_t kMtu = 1500;

    /// \brief Port for BEACON packets
    public: static const uint32_t kBeaconPort = 4000u;

    /// \brief Thread for triggering beacon transmits
    private: std::thread *beaconThread = nullptr;

    /// \brief The current list of neighbors
    ///
    /// Accumulates data on neighbors based on received packets,
    /// stores time of last receive and signal strength
    private: Neighbor_M neighbors;

    /// \brief An Ignition Transport node for communications.
    private: ignition::transport::Node node;

    private: using Callbacks =
      std::unordered_map<communication_broker::EndpointID, Callback_t>;

    /// \brief User callbacks. The key is the topic name
    /// (e.g.: "/subt/192.168.2.1/4000") and the value is the user callback.
    private: std::map<std::string, Callbacks> callbacks;

    /// \brief True when the broker validated my address. Enabled must be true
    /// for being able to send and receive data.
    private: bool enabled = false;

    /// \brief True when the client has advertised the OnMessage
    /// function to the broker already.
    private: bool advertised = false;

    /// \brief When true, the Ignition service will only be visible within
    /// this process.
    private: bool isPrivate = false;

    /// \brief A mutex for avoiding race conditions.
    private: mutable std::mutex mutex;

    /// \brief Subscriber that receives comms messages from SubtRosRelay.
    private: ros::Subscriber commsSub;

    /// \brief Clock message from simulation. Used by the base station.
    /// The base station is run as a plugin alongside simulation, and does
    /// not have access to ros::Time.
    private: ignition::msgs::Clock clockMsg;

    /// \brief Mutex to protect the clockMsg.
    private: std::mutex clockMutex;

    /// \brief True if this is the base station.
    private: bool useIgnition = false;

    /// \brief True if the beacon is running.
    private: bool beaconRunning{true};

    /// \brief Period of the beacon in nanoseconds.
    private: int64_t beaconPeriodNs{0};

    private: communication_broker::ClientID clientId {
      communication_broker::invalidClientId
    };
  };
}

/// \brief Backwards compatibility that allows easily testing Bind() success.
/// \param _val Result of a Bind() operation.
/// \return True if the Bind() operation failed.
bool operator !(const std::vector<
  std::pair<subt::communication_broker::EndpointID, std::string>>& _val)
{
  return _val.empty();
}

#endif
