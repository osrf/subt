# SUBT Virtual Communication System

*Proposal for refactoring of SUBT communication system*

The SUBT virtual communication system is a set of libraries that can be used to
efficiently simulate representative multi-agent communication in a SUBT
environment. There are three components to this system and a Gazebo plugin that
exercises these components to provide real-time simulation capabilities:

- **RF Model**: Given location of transmitter and receiver, transmit power,
  antenna specifications, and representation of the environment, computes
  the received signal (dBm) at the receiver.

- **Communication Model**: Computes success for each attempted communication
  based on **RF model**, radio specification, and communication channel usage.

- **Communication Broker**: Centralized node that receives communication
  requests via client API. Queries **communication model** and forwards data
  accordingly.

Implementations of each of these components are provided with the intention that
functionality is modularized such that alternative instantiations may exist.

## RF Model

Provides function of the form

``` c++
struct radio_state
{
  geometry_msgs::Pose pose; // Location in world frame
  uint64_t id;              // Unique radio ID
  double antenna_gain;      // Isotropic antenna gain
};

double received_power(double tx_power,
                      const radio_state& tx_state,
                      const radio_state& rx_state);

```

For efficiency, the RF model implementation could:

- Assume constant transmit power, periodically compute the received power, and
  maintain a complete received-power graph for all nodes.
- Compute received power in an event-based paradigm but quantize results and
  provide cached computations.

## Communication Model

Provides function of the form

```c++
struct channel_config
{
  double capacity;         // Bits-per-second
  double default_tx_power; // dBm
  string modulation;       // E.g., QPSK
};

bool attempt_send(const channel_config& channel,
                  const radio_state& tx_state,
                  const radio_state& rx_state);
```

The **communication model** is responsible for managing shared channels
and, if requests exceed channel capacity, the associated queues. *The
simplest approach being: drop all packets that exceed channel
capacity.*

## Communication Broker

Responsible for:

- Sending data between nodes
- Managing neighbor lists for each node

At initialization (or periodically), provided reference to Gazebo to
"discover" all radio models and associated parameters (transmit power,
antenna gain, frequency, coding, capacity, etc).

Interfaces via publish/subscribe with each CommsClient.  On request to
send data from source `i` to destination `j`:

1. Queries the physical location of each agent
2. Looks up radio properties
3. Calls `attempt_send` and forwards data accordingly
