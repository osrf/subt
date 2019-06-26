# SUBT Virtual Communication System

The SUBT virtual communication system is a set of libraries that can
be used to efficiently simulate representative multi-agent
communication in a SUBT environment. There are three server-side
components to this system and a Gazebo plugin that exercises these
components to provide real-time simulation capabilities:

- **RF Interface/Model**: Given location of transmitter and receiver,
  transmit power, antenna specifications, and representation of the
  environment, computes the received signal (dBm) at the receiver.

- **Communication Model**: Computes success for each attempted communication
  based on **RF model**, radio specification, and communication channel usage.

- **Communication Broker**: Centralized node that receives communication
  requests via client API. Queries **communication model** and forwards data
  accordingly.

The **Communication Client** is a distributed helper class that
  communicates send requests with the **communication broker** and
  provides an API for registering callbacks when successful
  communications are received.

Implementations of each of these components are provided with the
intention that functionality is modularized such that alternative
instantiations may exist. High-level documentation is provided below
and inline documentation is provided in the code for API reference.

## RF Interface/Model

The [RF
interface](subt_rf_interface/include/subt_rf_interface/subt_rf_interface.h)
defines the radio state that will be tracked along with a received
signal power type and function signature that must be implemented for
an RF model.

``` c++
struct radio_state
{
  geometry_msgs::PoseStamped pose; // Location
  std::list<std::pair<ros::Time, uint64_t>> bytes_sent;
  uint64_t bytes_sent_this_epoch;
  std::list<std::pair<ros::Time, uint64_t>> bytes_received;
  uint64_t bytes_received_this_epoch;
  double antenna_gain;      // Isotropic antenna gain
};

struct rf_power
{
  double mean;
  double variance;
  operator double() { return mean; }
};


typedef std::function<rf_power(const double&, // tx_power
                               radio_state&, // tx_state
                               radio_state&  //rx_state
                               )> pathloss_function;

```

Current implementations of the RF model include:

- [distance_based_received_power](subt_rf_interface/include/subt_rf_interface/subt_rf_interface.h#lines-40).
Binary received power based on maximum range.
- [log_normal_received_power](subt_rf_interface/include/subt_rf_interface/subt_rf_interface.h#lines-45).
Log-normal fading based on range.
- [VisibilityModel](subt_gazebo_los_model/include/subt_gazebo_los_model/visibility_rf_model.h#lines52)
Log-normal fading based on range with exponent set by visibility heuristic.

Note, some RF models may require information about the simulated
environment but the function to compute received power should not take
this as an argument. For efficiency, the RF model implementation
can:

- Assume constant transmit power, periodically compute the received power, and
  maintain a complete received-power graph for all nodes.
- Compute received power in an event-based paradigm but quantize results and
  provide cached computations.

## Communication Model

Provides function of the form

```c++
struct radio_configuration
{
  double capacity;         // Bits-per-second
  double default_tx_power; // dBm
  std::string modulation;       // E.g., QPSK
  double noise_floor;      // dBm
  rf_interface::pathloss_function pathloss_f; // Function for computing pathloss
};

bool attempt_send(const channel_config& channel,
                  radio_state& tx_state,
                  radio_state& rx_state,
                  const uint64_t& num_bytes);
```

The **communication model** is responsible for managing shared
channels and, if requests exceed channel capacity, the associated
queues. *The simplest approach being: drop all packets that exceed
channel capacity.* The [default
implementation](subt_communication_model/include/subt_communication_model/subt_communication_model.h)
does this.

## Communication Broker

Responsible for:

- Sending data between nodes
- Managing neighbor lists for each node (*deprecated?*)

At initialization, the **communication broker** is given:

- a default radio configuration which includes a function handle for
  evaluating path loss,
- a function handle for the communication model to evaluate attempted sends
- a function handle to query the simulation for the current state of
  an agent

The **communication broker** interfaces via Ignition transport with
each **communication client** and on request to send data from source
`i` to destination `j`:

1. Queries the physical location of each agent
2. Looks up radio properties
3. Calls `attempt_send` and forwards data accordingly

### Communication Broker Gazebo Plugin

The
[subt_gazebo/CommsBrokerPlugin](../subt_gazebo/src/CommsBrokerPlugin.cc)
is a Gazebo plugin that instantiates the **communication broker, RF
model, and communication model** in a running Gazebo instance,
providing pose updates and access to the environment as needed.

## Communication Client

The
[CommsClient](subt_communication_broker/include/subt_communication_broker/subt_communication_client.h)
provides an interface to `bind` callback functions to certain "ports"
for handling received communication and an `SendTo` function for
attempting UDP-like unreliable transport of data. It communicates with
the centralized **communication broker** via Ignition transport.

The communication client keeps track of recent successfully received
messages in order to report neighbor state information (a map from
remote address to time of reception and received signal strength in
dBm). Additionally, the client class can be configured to periodically
send a beacon (broadcast) packet to stimulate neighbor reporting.

## SUBT Comms Test

Curses-based python script for testing SUBT communication system
performance. The script is meant to be run in a centralized setting
(i.e., simulation + each robot's controller on the same ROS master)
and requires that the `subt_example/subt_example_node` be running to
provide the `create_peer` service required by the test script.

To execute, for example:

```bash
# Bring up simulator with two robots
ign launch -v 4 virtual_stix.ign robotName1:=X1 robotConfig1:=X1_SENSOR_CONFIG_1 robotName2:=X2 robotConfig2:=X1_SENSOR_CONFIG_1
```

In another terminal
``` bash
# Launch X1 example node
roslaunch subt_example example_robot.launch name:=X1
```

In yet another terminal
``` bash
# Launch X2 example node
roslaunch subt_example example_robot.launch name:=X2
```

Finally, run the following in a fourth terminal

``` bash
rosrun subt_comms_test subt_comms_tester.py
```
The `sub_comms_tester.py` script will detect agents running the
`subt_example_node` and configure appropriately. The shell-interface
provides functionality for verifying inter-agent
messages and computing statistics, e.g., dropped packets, tx/rx rates,
latency, and received signal strength.
