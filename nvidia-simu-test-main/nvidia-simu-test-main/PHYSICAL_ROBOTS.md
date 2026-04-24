# From Simulation to Physical Robots — Technical Guide

This document explains how the ROS2 obstacle avoidance system built in this project can be transferred to physical robot hardware with **zero code changes** to the control node. It covers compatible platforms, sensor integration, DDS networking, driver architecture, and the exact points where simulation and reality diverge.

---

## The Sim-to-Real Promise

The `obstacle_avoidance_node.py` in this project has exactly two ROS2 interfaces:

```
Subscribes: /scan  (sensor_msgs/LaserScan)
Publishes:  /cmd_vel (geometry_msgs/Twist)
```

It knows nothing about Isaac Sim, OmniGraph, PhysX, or Docker. It receives a laser scan, computes a steering command, and publishes a velocity. **Any robot that publishes `LaserScan` on `/scan` and accepts `Twist` on `/cmd_vel` is a drop-in replacement for the simulation.** This is the entire point of ROS2's message abstraction layer.

The same `dashboard_server.py` would also work unchanged — it subscribed to `/odom`, `/scan`, `/cmd_vel`, and `/clock` using standard message types.

---

## ROS2 Topic System — How Messages Actually Move

### What a "Topic" Is

A ROS2 topic is a **named, typed, many-to-many message channel**. There is no server, no broker, no central registry. A topic is just an agreed-upon string (like `"/scan"`) paired with a message type (like `sensor_msgs/msg/LaserScan`). Any node can publish to it. Any node can subscribe to it. They don't need to know about each other.

When the obstacle avoidance node does this:

```python
self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
self.pub_cmd  = self.create_publisher(Twist, '/cmd_vel', 10)
```

...it is telling the DDS middleware: "I want to read `LaserScan` messages on `"/scan"` and write `Twist` messages on `"/cmd_vel"`." DDS handles the rest — finding the publisher, negotiating QoS, and delivering data.

### The DDS Layer Underneath

ROS2 does not implement its own networking. It delegates all communication to **DDS (Data Distribution Service)**, an OMG standard for real-time publish/subscribe. In this project we use **FastDDS** (also called Fast-RTPS), but CycloneDDS and Connext DDS are also options. The ROS2 ↔ DDS mapping:

| ROS2 Concept | DDS Concept | What It Actually Is |
|-------------|------------|-------------------|
| Topic name `/scan` | DDS Topic `rt/scan` | String identifier (ROS2 prefixes `rt/`) |
| Message type `LaserScan` | DDS Type `sensor_msgs::msg::dds_::LaserScan_` | IDL-generated type with CDR serialization |
| Publisher | DDS DataWriter | Writes serialized bytes to the DDS bus |
| Subscription | DDS DataReader | Reads serialized bytes from the DDS bus |
| QoS policy | DDS QoS | Reliability, durability, history depth, deadline |
| Node | DDS Participant (or sub-entity) | A process with a unique GID on the domain |
| Domain ID | DDS Domain ID | Integer (default 0) that isolates groups |

### Step-by-Step: What Happens When `/scan` Is Published

Let's trace exactly what happens when Isaac Sim's OmniGraph publishes one LaserScan message, and the obstacle avoidance node receives it:

#### 1. Discovery (happens once, at startup)

When a node starts and creates a publisher or subscription, DDS announces it via **Simple Participant Discovery Protocol (SPDP)**:

```
Isaac Sim container starts → ROS2Context OmniGraph node creates DDS Participant
  → Participant sends SPDP announcement via UDP multicast to 239.255.0.1:7400
  → Announcement contains: participant GUID, unicast locators (IP:port), domain ID

Sidecar container starts → obstacle_avoidance_node creates DDS Participant  
  → Same multicast announcement

Both participants receive each other's announcements
  → They now know each other's unicast addresses
```

Next comes **Simple Endpoint Discovery Protocol (SEDP)** — participants exchange their reader/writer endpoints:

```
Isaac Sim: "I have a DataWriter for topic 'rt/scan', type LaserScan_, BEST_EFFORT QoS"
Sidecar:   "I have a DataReader for topic 'rt/scan', type LaserScan_, BEST_EFFORT QoS"

DDS matches the writer and reader:
  → Topic names match ✓
  → Type names match ✓
  → QoS compatible (both BEST_EFFORT) ✓
  → Match established → data will flow
```

This entire exchange happens over UDP in the first ~1–2 seconds after node startup. No configuration needed (when on the same network with multicast support).

#### 2. Serialization (at the publisher)

When the OmniGraph `PublishLaserScan` node fires on a frame tick:

```
OmniGraph gathers lidar data from IsaacReadLidar outputs:
  linearDepthData = float32[360]    (raw depth values from PhysX raycast)
  azimuthRange = [-180.0, 180.0]    (degrees)
  horizontalFov = 360.0
  horizontalResolution = 1.0
  numCols = 360
  ...

The ROS2 bridge node constructs a sensor_msgs::msg::LaserScan:
  header.stamp = simulation_time
  header.frame_id = "lidar"
  angle_min = -π
  angle_max = +π
  angle_increment = 2π / 360
  ranges = [d0, d1, d2, ... d359]    (metres)

CDR serialization (Common Data Representation):
  The message struct is serialized to a byte buffer using XCDR (Extended CDR):
  
  Bytes:
  [00 01 00 00]              → CDR encapsulation header (little-endian, CDR1)
  [xx xx xx xx xx xx xx xx]  → header.stamp (sec: int32 + nanosec: uint32)
  [05 00 00 00]              → frame_id string length = 5
  [6C 69 64 61 72 00 00 00]  → "lidar\0" + padding to 4-byte alignment
  [DB 0F 49 C0]              → angle_min = -π (float32 LE IEEE 754)
  [DB 0F 49 40]              → angle_max = +π
  [2E BD 4B 3C]              → angle_increment ≈ 0.01745 (2π/360)
  [00 00 00 00]              → time_increment = 0.0
  [00 00 00 00]              → scan_time = 0.0
  [CD CC 4C 3E]              → range_min = 0.2
  [00 00 20 41]              → range_max = 10.0
  [68 01 00 00]              → ranges array length = 360
  [xx xx xx xx] × 360        → 360 float32 values = 1440 bytes
  [68 01 00 00]              → intensities array length = 360
  [xx xx xx xx] × 360        → 360 float32 values = 1440 bytes
  
  Total ≈ 2920 bytes payload + headers
```

CDR is a binary format — not JSON, not protobuf, not msgpack. It has zero-overhead deserialization for aligned types because the wire format matches the in-memory struct layout on little-endian architectures.

#### 3. Transport (over the network)

FastDDS takes the serialized buffer and hands it to the transport layer:

```
Serialized LaserScan (2920 bytes)
         │
         ▼
┌─────────────────────────────────┐
│  RTPS (Real-Time Publish-       │
│  Subscribe Protocol) Layer      │
│                                 │
│  Wraps in RTPS Data submessage: │
│  ┌────────────────────────────┐ │
│  │ RTPS Header (20 bytes)     │ │
│  │  protocol: "RTPS"          │ │
│  │  version: 2.3              │ │
│  │  vendor: eProsima          │ │
│  │  guidPrefix: [12 bytes]    │ │
│  ├────────────────────────────┤ │
│  │ InfoTimestamp submessage    │ │
│  │ InfoDestination submessage │ │
│  ├────────────────────────────┤ │
│  │ Data submessage            │ │
│  │  writerEntityId            │ │
│  │  readerEntityId            │ │
│  │  sequenceNumber            │ │
│  │  serializedPayload:        │ │
│  │    [CDR bytes above]       │ │
│  └────────────────────────────┘ │
└─────────────────────────────────┘
         │
         ▼
   Transport Layer
         │
    ┌────┴────┐
    │  UDP    │  ← In our project (forced by FastDDS XML config)
    │  socket │
    └────┬────┘
         │
    sendto(subscriber_ip:port, rtps_packet)
```

**In our Docker setup**, this is a `sendto()` on a UDP socket to `127.0.0.1:<port>` because all containers share the host network namespace (`--net=host`). The kernel copies bytes from the publisher's socket buffer to the subscriber's socket buffer. On a physical robot with WiFi, this would be a real network packet.

**Shared-memory path (default, when it works):**

On a single machine without Docker IPC isolation, FastDDS prefers shared memory:

```
Publisher process                    Subscriber process
      │                                    │
      ▼                                    ▼
 mmap /dev/shm/fastrtps_segment_xxx   mmap same segment
      │                                    │
      ├─ Write CDR bytes to ring buffer ──►├─ Read CDR bytes from ring buffer
      │                                    │
      └─ Signal via condition variable ───►└─ Wake up, process message
```

This is why shared-memory transport gives near-zero-copy performance (~1 µs per message) — the data is never copied through the kernel's network stack. This is also why it broke in our Docker setup: containers with `ipc=private` get separate `/dev/shm` namespaces, so the shared segment is invisible.

#### 4. Deserialization (at the subscriber)

When the obstacle avoidance node's DDS DataReader receives the RTPS packet:

```
UDP recv() → RTPS deserialization → CDR buffer extracted
         │
         ▼
rclpy executor runs on spin thread:
  1. DDS notifies executor: "new data on /scan DataReader"
  2. Executor calls: _take_message() → gets raw CDR bytes from DDS
  3. CDR deserializer maps bytes back into Python LaserScan object:
     
     msg = LaserScan()
     msg.header.stamp.sec = struct.unpack('<i', buf[offset:offset+4])
     msg.header.stamp.nanosec = struct.unpack('<I', buf[offset+4:offset+8])
     msg.header.frame_id = buf[offset:offset+length].decode('utf-8')
     ...
     msg.ranges = array.array('f', buf[offset:offset+360*4])  # zero-copy slice
     
  4. Executor invokes user callback:
     self.scan_cb(msg)
     
  5. User callback runs:
     self.ranges = list(msg.ranges)  # obstacle_avoidance_node stores the scan
```

In Python (rclpy), there is an intermediate C layer (`_rclpy`) that handles the CDR deserialization in C for performance, then constructs the Python message object. In C++ (rclcpp), the message is deserialized directly into a shared pointer to the message struct — essentially a memcpy from the CDR buffer for flat types.

#### 5. The Full Round Trip

Putting it all together for one control cycle:

```
Time  Event                                    Where
─────────────────────────────────────────────────────────────────────
t+0.000  PhysX simulates lidar raycasts         Isaac Sim (GPU)
t+0.001  IsaacReadLidar outputs 360 depths      OmniGraph
t+0.002  PublishLaserScan serializes + publishes OmniGraph → DDS
t+0.003  UDP packet sent to sidecar             Host network
t+0.003  DDS DataReader receives packet          Sidecar container
t+0.004  CDR deserialized → scan_cb fires        obstacle_avoidance_node
t+0.004  scan_cb stores ranges                   Python variable
...
t+0.100  Timer fires control_loop                obstacle_avoidance_node
t+0.101  Gap-finding + steer computation         Python
t+0.102  cmd_vel Twist serialized + published    obstacle_avoidance → DDS
t+0.103  UDP packet sent to Isaac Sim            Host network
t+0.103  SubscribeTwist DataReader receives      OmniGraph
t+0.104  DiffController computes wheel speeds    OmniGraph
t+0.104  ArticController applies to joints       PhysX
t+0.105  Robot wheels turn in simulation         PhysX
─────────────────────────────────────────────────────────────────────
Total round trip: ~100 ms (dominated by 10 Hz control timer)
```

### Topic Naming and Namespacing

Topic names are strings with `/` separators, similar to file paths:

| Name | Convention |
|------|-----------|
| `/scan` | Global — any node can see it |
| `/robot1/scan` | Namespaced — multiple robots on same DDS domain |
| `~/scan` | Private — expands to `/<node_name>/scan` |

In a multi-robot scenario, you'd namespace each robot:

```bash
# Robot 1:
ROS_NAMESPACE=robot1 python3 obstacle_avoidance_node.py
# Subscribes to /robot1/scan, publishes /robot1/cmd_vel

# Robot 2:
ROS_NAMESPACE=robot2 python3 obstacle_avoidance_node.py
# Subscribes to /robot2/scan, publishes /robot2/cmd_vel
```

Same code, different namespace, no interference. The Isaac Sim bridge would need matching topic names in OmniGraph configuration.

### QoS: Why RELIABLE vs BEST_EFFORT Matters

DDS Quality of Service policies are negotiated during endpoint matching. The critical one for this project:

**Reliability:**

| Publisher QoS | Subscriber QoS | Result |
|--------------|----------------|--------|
| BEST_EFFORT | BEST_EFFORT | ✓ Data flows. Dropped packets are lost forever. |
| RELIABLE | RELIABLE | ✓ Data flows. DDS retransmits lost packets. |
| RELIABLE | BEST_EFFORT | ✓ Data flows (subscriber accepts weaker guarantee). |
| **BEST_EFFORT** | **RELIABLE** | **✗ No match. Zero data flows.** |

Isaac Sim publishes BEST_EFFORT. If you subscribe with the default (RELIABLE in rclpy), DDS refuses to match the endpoints. The subscription silently receives nothing. There is no error message. This is the most common ROS2 debugging trap.

The `10` in `create_subscription(LaserScan, '/scan', self.scan_cb, 10)` is a shorthand for `QoSProfile(depth=10)` with system defaults (RELIABLE reliability). To match BEST_EFFORT publishers:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)
self.create_subscription(LaserScan, '/scan', self.scan_cb, sensor_qos)
```

**Other QoS policies:**

| Policy | Options | Effect |
|--------|---------|--------|
| **Durability** | VOLATILE / TRANSIENT_LOCAL | TRANSIENT_LOCAL: late subscribers get the last N messages |
| **History** | KEEP_LAST(N) / KEEP_ALL | How many messages to buffer before overwriting |
| **Deadline** | Duration | Fail if no message within interval |
| **Lifespan** | Duration | Messages older than this are discarded |
| **Liveliness** | AUTOMATIC / MANUAL | How to detect if a publisher died |

### Memory and Performance

What actually happens in memory when 4 subscribers listen to `/scan` at 18 Hz with 360 rays (≈3 KB per message):

```
Publisher (Isaac Sim):
  Serializes once → 3 KB CDR buffer
  Sends to 4 subscribers (4 × sendto() on UDP, or 4 × SHM write)

Per subscriber (per message):
  UDP recv():          3 KB kernel → userspace copy
  CDR deserialization: 3 KB buffer → message struct
  Callback execution:  message struct processed
  
  Total per subscriber: ~6 KB memory bandwidth per message
  At 18 Hz: ~108 KB/s per subscriber
  With 4 subscribers: ~432 KB/s total

For /odom at 50 Hz (much smaller message, ~200 bytes):
  50 × 200 × 4 subscribers = 40 KB/s
```

On a Raspberry Pi 4, the DDS overhead for these four topics is well under 1% of CPU and negligible memory. The obstacle avoidance Python code itself is the bottleneck, not DDS.

**Zero-copy transport (FastDDS with SHM + data sharing):**

For large messages (images, point clouds), FastDDS can use zero-copy: the publisher writes directly to shared memory, and subscribers read from the same buffer without any copy. This requires `ipc=host` (or same IPC namespace) and is activated via:

```xml
<data_sharing>
  <kind>AUTOMATIC</kind>
  <shared_memory_directory>/dev/shm</shared_memory_directory>
</data_sharing>
```

Not used in this project due to Docker IPC isolation, but critical for camera streams on physical robots (a 1080p image at 30 Hz = ~186 MB/s — you absolutely need zero-copy).

### Introspection: The ROS2 CLI

The ROS2 CLI tools are themselves DDS participants. When you run:

```bash
ros2 topic list
```

...it creates a temporary DDS Participant, waits ~2 seconds for SPDP/SEDP discovery, collects all discovered topics, prints them, and exits. Other tools:

```bash
# Show message type and publisher/subscriber counts:
ros2 topic info /scan
# Type: sensor_msgs/msg/LaserScan
# Publisher count: 1
# Subscription count: 2

# Measure publish rate (creates a BEST_EFFORT subscriber, counts messages):
ros2 topic hz /scan
# average rate: 18.2 Hz

# Print one message (deserializes CDR → YAML):
ros2 topic echo /scan --once
# header:
#   stamp:
#     sec: 1234
#     nanosec: 567000000
#   frame_id: lidar
# angle_min: -3.14159...
# ranges: [1.2, 1.5, 0.8, ...]

# Publish a single message (serializes YAML → CDR):
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.1}}"
```

These tools work identically whether the publisher is Isaac Sim, a physical lidar, or a rosbag replay. That's the whole point.

---

## Compatible Physical Platforms

### NVIDIA Jetbot (Direct Match)

The simulated robot is a Jetbot, and the physical [NVIDIA Jetbot](https://github.com/NVIDIA-AI-IOT/jetbot) exists as real hardware:

| Component | Simulation | Physical |
|-----------|-----------|----------|
| Compute | Isaac Sim on RTX A4000 | NVIDIA Jetson Nano / Orin Nano |
| Chassis | USD mesh, PhysX collision | Adafruit 3-layer chassis |
| Drive | `DifferentialController` OG node | 2× TT DC motors via Adafruit MotorHAT (I²C) |
| Wheels | `left_wheel_joint` / `right_wheel_joint` | 60mm wheels, ~0.0325 m radius |
| Wheel base | 0.1125 m (OG parameter) | ~0.1125 m (physical measurement) |
| Lidar | PhysX raycast `RangeSensorCreateLidar` | Add-on LD06/LD19 or RPLidar A1 |
| Camera | USD Camera prim (not used for avoidance) | CSI camera via V4L2 |

The physical Jetbot doesn't ship with a lidar — you'd mount one on the chassis. The control stack is identical.

**ROS2 driver for Jetbot motors:**

The Jetbot's motor driver is an I²C motor controller (PCA9685 or compatible). The standard approach:

```
/cmd_vel (Twist) → jetbot_driver_node → I²C → MotorHAT → DC motors
```

A minimal driver node:

```python
class JetbotDriver(Node):
    def __init__(self):
        super().__init__('jetbot_driver')
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.robot = Robot()  # jetbot.Robot() — controls MotorHAT via I2C

    def cmd_cb(self, msg):
        # Convert Twist to differential drive
        linear = msg.linear.x    # m/s forward
        angular = msg.angular.z  # rad/s yaw

        # Differential drive kinematics (same as DifferentialController OG node)
        wheel_base = 0.1125  # metres — must match simulation
        left_speed  = linear - angular * wheel_base / 2.0
        right_speed = linear + angular * wheel_base / 2.0

        # Normalize to [-1, 1] motor range
        max_speed = 0.5  # max m/s of physical robot
        self.robot.left_motor.value  = clamp(left_speed / max_speed, -1, 1)
        self.robot.right_motor.value = clamp(right_speed / max_speed, -1, 1)
```

### TurtleBot3 Burger / Waffle

The [TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) is the most widely deployed ROS2 development robot and is a near-perfect match for this project's interface:

| Feature | Our Simulation | TurtleBot3 Burger | TurtleBot3 Waffle Pi |
|---------|---------------|-------------------|---------------------|
| Drive | Differential | Differential (2× Dynamixel XL430) | Differential (2× Dynamixel XL430) |
| Lidar | PhysX 360° | LDS-01 360° (1800 pts/rev, 5 Hz) | LDS-02 360° (2300 pts/rev, 5 Hz) |
| Compute | Isaac Sim container | Raspberry Pi 4 or Jetson | Raspberry Pi 4 or Jetson |
| `/cmd_vel` | Twist, 10 Hz from sidecar | Twist, native support | Twist, native support |
| `/scan` | LaserScan, 360 rays, ~18 Hz | LaserScan, ~1800 pts, 5 Hz | LaserScan, ~2300 pts, 5 Hz |
| `/odom` | OmniGraph ComputeOdom | `robot_state_publisher` from wheel encoders | Same + IMU fusion |

**TurtleBot3 already publishes `/scan` and subscribes to `/cmd_vel` out of the box.** To run our obstacle avoidance:

```bash
# On the TurtleBot3 (Raspberry Pi / Jetson):
ros2 launch turtlebot3_bringup robot.launch.py  # starts motors + lidar + odom

# On any machine on the same network:
python3 obstacle_avoidance_node.py
```

The only parameter adjustment needed:

```bash
ros2 run obstacle_avoidance obstacle_avoidance_node \
    --ros-args -p linear_speed:=0.15 -p ray_range:=3.5
```

(TurtleBot3 Burger tops out at ~0.22 m/s; the LDS lidar max range is 3.5 m.)

### TurtleBot4 (iRobot Create 3)

The [TurtleBot4](https://turtlebot.github.io/turtlebot4-user-manual/) uses an iRobot Create 3 base with an RPLIDAR A1 and Raspberry Pi 4:

| Feature | Value |
|---------|-------|
| Lidar | RPLIDAR A1M8 (360°, 8000 pts/sec, 5.5 Hz, 12 m range) |
| Drive | iRobot Create 3 differential (built-in motor controller) |
| Interface | Standard `/scan` (LaserScan) + `/cmd_vel` (Twist) |
| Compute | Raspberry Pi 4 (4 GB) |
| Nav stack | Nav2 pre-installed |

Drop-in compatible. Our node runs unchanged.

### Custom Differential-Drive Robot

Any differential-drive robot with a 2D lidar works if it provides these two topic interfaces. Common DIY builds:

| Component | Options | ROS2 Driver |
|-----------|---------|-------------|
| Motors | Pololu 37D gearmotors, NEMA17 steppers | `ros2_control` + hardware interface |
| Motor controller | L298N, TB6612, ODrive, Roboclaw | Respective ROS2 driver packages |
| SBC | Raspberry Pi 4/5, Jetson Nano/Orin, Intel NUC | Any Linux running ROS2 Humble |
| Lidar | RPLidar A1/A2/A3, LD06/LD19, Hokuyo URG, SICK TiM | See lidar section below |
| Wheel encoders | Hall effect, optical quadrature | `ros2_control` encoder interface |

---

## Physical Lidar Integration — Deep Technical Detail

### How a Physical 2D Lidar Works

A 2D lidar rotates a laser rangefinder (typically 905 nm or 850 nm) through 360°. On each revolution:

1. **Laser pulse emission:** A near-infrared laser diode fires a short pulse (~5 ns for time-of-flight sensors, or continuous for triangulation).
2. **Distance measurement:**
   - **Time-of-flight (ToF):** Measures the round-trip time of the laser pulse. At the speed of light (299,792,458 m/s), 1 ns ≈ 15 cm round-trip. Example: RPLidar A1 (triangulation), Hokuyo URG-04LX (ToF).
   - **Triangulation:** A camera sensor detects the reflected dot at an angle proportional to distance. Cheaper but limited to ~12 m range. Example: RPLidar A1M8.
   - **Phase-shift:** Modulates the laser and measures the phase difference of the return signal. Higher precision at longer ranges. Example: SICK TiM series.
3. **Angular encoding:** A rotary encoder (optical or magnetic) records the exact angle of each measurement. Resolution depends on sample rate ÷ rotation speed (e.g., 8000 samples/sec ÷ 5.5 rotations/sec ≈ 1454 points/revolution).
4. **Data transmission:** The lidar sends measurement packets over **UART (serial)**, **USB**, or **Ethernet** to the host computer.

### Wire Protocol: RPLidar A1 as Example

The RPLidar A1M8 (most common hobby lidar, ~$100) communicates over UART at 115200 baud:

**Request/Response Commands:**

| Command | Code | Description |
|---------|------|-------------|
| `GET_HEALTH` | `0x52` | Check device status |
| `GET_INFO` | `0x50` | Model, firmware, serial number |
| `SCAN` | `0x20` | Start continuous scanning |
| `EXPRESS_SCAN` | `0x82` | High-density scan mode |
| `FORCE_SCAN` | `0x21` | Scan ignoring motor speed |
| `STOP` | `0x25` | Stop scanning |
| `RESET` | `0x40` | Reboot device |

**Scan Data Packet Format (legacy scan mode):**

```
Byte 0: quality[7:2] | start_flag_inverse | start_flag
Byte 1: angle[6:0] | check_bit (always 1)
Byte 2: angle[14:7]
Byte 3: distance[7:0]
Byte 4: distance[15:8]
```

- **quality:** 0–47 (signal strength, 0 = invalid measurement)
- **start_flag:** 1 = first measurement of a new revolution
- **angle:** 0–360° in Q6 fixed-point (value / 64.0 = degrees)
- **distance:** millimetres (0 = invalid / out of range)

**Express Scan Packet (higher throughput):**

Uses cabin-based encoding where each packet contains a header + 16 measurement cabins. Each cabin stores two measurements with differential angle encoding, achieving ~4000+ measurements per revolution.

### From Wire Protocol to `/scan` Topic

The ROS2 driver chain for a physical lidar:

```
┌──────────┐   UART/USB    ┌───────────────────┐   ROS2 DDS   ┌───────────────────┐
│ RPLIDAR  │──────────────→│  rplidar_ros2     │─────────────→│ /scan topic       │
│ hardware │  115200 baud  │  (driver node)    │              │ (LaserScan msg)   │
│          │  5-byte pkts  │                   │              │                   │
└──────────┘               │  1. Open serial   │              │ angle_min: -π     │
                           │  2. Send SCAN cmd │              │ angle_max: +π     │
                           │  3. Read packets  │              │ angle_increment:  │
                           │  4. Accumulate    │              │   2π/num_readings │
                           │     one revolution│              │ ranges: float32[] │
                           │  5. Build msg     │              │ intensities: []   │
                           │  6. Publish       │              │                   │
                           └───────────────────┘              └───────────────────┘
```

**Driver node internals (rplidar_ros2 package):**

```cpp
// Simplified driver loop (C++ for performance)
void RPLidarNode::scan_loop() {
    rplidar_response_measurement_node_hq_t nodes[8192];
    size_t count = _countof(nodes);

    // SDK call: blocks until one full 360° revolution is complete
    drv->grabScanDataHq(nodes, count);

    // Sort by angle (hardware may deliver out-of-order)
    drv->ascendScanData(nodes, count);

    // Build LaserScan message
    auto msg = sensor_msgs::msg::LaserScan();
    msg.header.stamp = this->now();
    msg.header.frame_id = "laser";
    msg.angle_min = 0.0;
    msg.angle_max = 2.0 * M_PI;
    msg.angle_increment = 2.0 * M_PI / count;
    msg.range_min = 0.15;  // 15 cm minimum
    msg.range_max = 12.0;  // 12 m maximum
    msg.time_increment = scan_duration / count;
    msg.scan_time = scan_duration;

    msg.ranges.resize(count);
    msg.intensities.resize(count);
    for (size_t i = 0; i < count; i++) {
        // Convert from mm to metres, 0 = invalid
        float dist = nodes[i].dist_mm_q2 / 4000.0f;
        msg.ranges[i] = (dist < msg.range_min) ? 0.0f : dist;
        msg.intensities[i] = nodes[i].quality >> 2;
    }

    scan_publisher->publish(msg);
}
```

The resulting `LaserScan` message is **byte-for-byte identical in structure** to what Isaac Sim's `PublishLaserScan` OmniGraph node publishes. The obstacle avoidance node cannot tell the difference.

### Common 2D Lidar ROS2 Drivers

| Lidar | ROS2 Package | Connection | Rays/Rev | Rate | Max Range | Price |
|-------|-------------|------------|----------|------|-----------|-------|
| RPLidar A1M8 | `rplidar_ros` | USB-UART | ~1450 | 5.5 Hz | 12 m | ~$100 |
| RPLidar A2M12 | `rplidar_ros` | USB-UART | ~4000 | 10 Hz | 12 m | ~$300 |
| RPLidar A3 | `rplidar_ros` | USB-UART | ~8000 | 10 Hz | 25 m | ~$500 |
| SLAMTEC LD06 | `ldlidar_stl_ros2` | USB-UART | ~4500 | 10 Hz | 12 m | ~$30 |
| SLAMTEC LD19 | `ldlidar_stl_ros2` | USB-UART | ~4500 | 10 Hz | 12 m | ~$50 |
| Hokuyo URG-04LX | `urg_node2` | USB | 682 | 10 Hz | 5.6 m | ~$1000 |
| Hokuyo UST-10LX | `urg_node2` | Ethernet | 1080 | 40 Hz | 10 m | ~$1600 |
| SICK TiM571 | `sick_scan_xd` | Ethernet | 811 | 15 Hz | 25 m | ~$2500 |
| YDLidar X4 | `ydlidar_ros2_driver` | USB-UART | ~5000 | 7 Hz | 10 m | ~$70 |
| Velodyne VLP-16 | `velodyne_driver` | Ethernet | 28800 (3D) | 20 Hz | 100 m | ~$4000 |
| Intel L515 | `realsense2_camera` | USB3 | depth image→scan | 30 Hz | 9 m | ~$350 |

For a direct match to our 360-ray, 10 m simulation, the **LD06** ($30) or **RPLidar A1** ($100) are the best budget options.

### LaserScan Message Deep Dive

The `sensor_msgs/msg/LaserScan` message is the universal interface between any lidar and any consumer:

```
std_msgs/Header header
  builtin_interfaces/Time stamp    # Timestamp of first ray
  string frame_id                   # TF frame (e.g., "laser", "lidar")

float32 angle_min        # Start angle (radians), typically -π or 0
float32 angle_max        # End angle (radians), typically +π or 2π
float32 angle_increment  # Angular step between rays (radians)
float32 time_increment   # Time between rays (seconds) — for motion compensation
float32 scan_time        # Full revolution time (seconds)
float32 range_min        # Minimum valid distance (below = too close / invalid)
float32 range_max        # Maximum valid distance (above = no return / invalid)
float32[] ranges         # Distance array (metres). inf = no return. 0 = invalid.
float32[] intensities    # Signal strength (optional, driver-dependent)
```

**Memory layout on the wire (DDS CDR serialization):**

```
Offset  Size   Field
0x00    4+N    header.stamp.sec (int32) + nanosec (uint32) + frame_id (string)
...     4      angle_min (float32, little-endian IEEE 754)
...     4      angle_max
...     4      angle_increment
...     4      time_increment
...     4      scan_time
...     4      range_min
...     4      range_max
...     4      ranges.length (uint32) — number of rays
...     N×4    ranges[0..N-1] (float32 array, little-endian)
...     4      intensities.length (uint32)
...     M×4    intensities[0..M-1] (float32 array)
```

For 360 rays: `ranges` = 360 × 4 bytes = 1440 bytes. Total message ≈ 1500–1600 bytes including header. For 4500-ray lidars: ≈ 18 KB per message.

### Differences Between Simulated and Physical Lidar

| Aspect | Simulation (PhysX) | Physical (e.g., RPLidar A1) |
|--------|-------------------|---------------------------|
| **Noise** | Zero noise — perfect raycasts | ±1–3% distance noise, multipath reflections |
| **Invalid readings** | Rare (only if ray misses all geometry) | Common: black surfaces absorb IR, glass is transparent, outdoor sunlight blinds sensor |
| **Angular accuracy** | Perfect (exact commanded angle) | ±0.5° typical encoder jitter |
| **Scan rate** | Tied to sim FPS (typically 18 Hz from 20 Hz rotation at 60 FPS) | Fixed by motor speed (5.5 Hz for A1, 10 Hz for A2) |
| **Ray count** | Exactly 360 (1° resolution, configured) | Variable (1454 for A1, depends on motor RPM and sample rate) |
| **Range** | 0.2–10.0 m (configured) | 0.15–12.0 m (hardware limit) |
| **Motion distortion** | None (all rays captured at same sim instant) | Significant at high speeds — rays at 0° and 180° are captured 100 ms apart |
| **Cross-talk** | None | Multiple lidars can interfere with each other |
| **Surface dependency** | All materials reflect equally | Black surfaces: weak return. Retroreflectors: saturated. Glass: pass-through |

**To bridge this gap in simulation**, you can add synthetic noise:

```python
# Add to Isaac Sim lidar rays before publishing (or post-process in a ROS2 node)
import numpy as np
noise = np.random.normal(0, 0.02, len(ranges))  # ±2 cm Gaussian noise
ranges = ranges + noise * ranges  # Proportional noise (1-2% of distance)
# Randomly drop 2% of rays (simulate surface absorption)
drop_mask = np.random.random(len(ranges)) < 0.02
ranges[drop_mask] = float('inf')
```

---

## Twist Message and Motor Control

### Twist Message Structure

```
geometry_msgs/msg/Twist:
  Vector3 linear    # Linear velocity
    float64 x       # Forward (m/s)     ← used
    float64 y       # Lateral (m/s)     ← 0 for differential drive
    float64 z       # Vertical (m/s)    ← 0
  Vector3 angular   # Angular velocity
    float64 x       # Roll (rad/s)      ← 0
    float64 y       # Pitch (rad/s)     ← 0
    float64 z       # Yaw (rad/s)       ← used
```

For a differential-drive robot, only `linear.x` and `angular.z` are used. The conversion to individual wheel velocities:

```
v_left  = linear.x - angular.z × (wheel_base / 2)
v_right = linear.x + angular.z × (wheel_base / 2)
```

This is exactly what Isaac Sim's `DifferentialController` OmniGraph node computes, and what any physical robot's driver must implement.

### Odometry: Physical vs Simulated

In simulation, `ComputeOdometry` reads the ground-truth position directly from PhysX. On a physical robot, odometry comes from **wheel encoders** and optionally **IMU fusion**:

**Wheel encoder odometry:**

```python
# Typical differential-drive odometry computation (runs at encoder tick rate)
def update_odometry(left_ticks, right_ticks, dt):
    # Convert encoder ticks to wheel displacement
    dl = left_ticks * metres_per_tick    # e.g., 0.000153 m/tick for 2048 CPR encoder
    dr = right_ticks * metres_per_tick

    # Robot displacement
    d_centre = (dl + dr) / 2.0
    d_theta  = (dr - dl) / wheel_base

    # Update pose
    self.theta += d_theta
    self.x += d_centre * math.cos(self.theta)
    self.y += d_centre * math.sin(self.theta)

    # Publish nav_msgs/Odometry
    odom_msg.pose.pose.position.x = self.x
    odom_msg.pose.pose.position.y = self.y
    odom_msg.twist.twist.linear.x = d_centre / dt
    odom_msg.twist.twist.angular.z = d_theta / dt
```

**Key difference:** Simulated odometry has zero drift. Physical wheel odometry drifts ~2–5% over distance due to wheel slip, uneven surfaces, and encoder resolution. For long-term accuracy, physical robots augment with:

- **IMU (Inertial Measurement Unit):** Gyroscope provides drift-free yaw rate; accelerometer detects tilt. Fused via Extended Kalman Filter (`robot_localization` package).
- **Visual odometry:** Camera-based motion estimation (ORB-SLAM, RTAB-Map).
- **AMCL:** Particle filter localization against a known map using lidar.

---

## Network Architecture: Physical Robot

### Single-Machine Setup (small robot)

For a Jetbot or TurtleBot3 with a Jetson/Pi:

```
┌──────────────────────────────────────────┐
│  Jetson Nano / Raspberry Pi              │
│                                          │
│  ┌────────────────┐  ┌────────────────┐  │
│  │ rplidar_ros2   │  │ motor_driver   │  │
│  │ /dev/ttyUSB0   │  │ I2C / GPIO     │  │
│  │ pub: /scan     │  │ sub: /cmd_vel  │  │
│  └────────────────┘  └────────────────┘  │
│                                          │
│  ┌────────────────┐  ┌────────────────┐  │
│  │ odom_node      │  │ obstacle_      │  │
│  │ pub: /odom     │  │ avoidance_node │  │
│  │ (wheel encoder)│  │ (OUR CODE)     │  │
│  └────────────────┘  └────────────────┘  │
│                                          │
│  All nodes: same DDS domain, localhost   │
│  Zero network latency                    │
└──────────────────────────────────────────┘
```

All nodes run in the same Linux process space. DDS uses shared-memory transport (the opposite of our Docker SHM problem — here it works because there are no container boundaries).

### Multi-Machine Setup (offloaded compute)

For computationally heavy tasks (SLAM, deep learning), offload to a base station:

```
┌────────────────────────┐          WiFi / Ethernet         ┌─────────────────────────┐
│  Robot (Jetson Nano)   │◄────────────────────────────────►│  Base Station (PC/Mac)  │
│                        │          DDS Discovery            │                         │
│  rplidar_ros2 → /scan  │          (multicast :7400)        │  obstacle_avoidance_    │
│  motor_driver ← /cmd   │                                  │    node (OUR CODE)      │
│  odom_node → /odom     │          DDS Data                 │                         │
│                        │          (unicast UDP)            │  dashboard_server.py    │
│  ROS_DOMAIN_ID=0       │                                  │  rviz2                  │
│  (same domain)         │                                  │  nav2 stack             │
│                        │                                  │  ROS_DOMAIN_ID=0        │
└────────────────────────┘                                  └─────────────────────────┘
```

**DDS multicast discovery:** Both machines send multicast to `239.255.0.1:7400`. Once they discover each other, data flows via unicast UDP. No broker, no server — fully peer-to-peer.

**Latency considerations:** WiFi adds 1–10 ms latency. For 10 Hz obstacle avoidance this is negligible. For real-time 100 Hz control loops, use Ethernet or run on-robot.

### DDS Configuration for Multi-Machine

On both machines, set the same domain and optionally pin discovery peers:

```bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# Optional: pin to specific network interface
export FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/fastdds_config.xml
```

FastDDS XML for multi-machine with known IPs (avoids multicast issues on some WiFi):

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <participant profile_name="participant_profile" is_default_profile="true">
    <rtps>
      <builtin>
        <initialPeersList>
          <locator>
            <udpv4>
              <address>192.168.1.100</address>  <!-- Robot IP -->
            </udpv4>
          </locator>
          <locator>
            <udpv4>
              <address>192.168.1.200</address>  <!-- Base station IP -->
            </udpv4>
          </locator>
        </initialPeersList>
      </builtin>
    </rtps>
  </participant>
</profiles>
```

---

## The ros2_control Framework

On a physical robot, motor control typically uses the **ros2_control** framework rather than direct topic subscribers. This provides:

1. **Hardware Abstraction Layer (HAL):** A C++ plugin (`hardware_interface`) talks to the physical motor controller
2. **Controller Manager:** Loads and manages control plugins at runtime
3. **Diff Drive Controller:** Converts `/cmd_vel` Twist to individual wheel velocity commands

```
                    ros2_control architecture
                    
/cmd_vel ──→ [diff_drive_controller] ──→ [hardware_interface] ──→ Motors
                                                │
                                         I2C / SPI / UART / CAN
                                                │
                                     [Motor Controller Board]
                                     (L298N, ODrive, Roboclaw)
```

**URDF configuration (robot description):**

```xml
<ros2_control name="jetbot_control" type="system">
  <hardware>
    <plugin>jetbot_hardware/JetbotHardwareInterface</plugin>
    <param name="i2c_bus">/dev/i2c-1</param>
    <param name="motor_hat_address">0x60</param>
  </hardware>
  
  <joint name="left_wheel_joint">
    <command_interface name="velocity">
      <param name="min">-10.0</param>
      <param name="max">10.0</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  
  <joint name="right_wheel_joint">
    <command_interface name="velocity">
      <param name="min">-10.0</param>
      <param name="max">10.0</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

**Controller configuration (YAML):**

```yaml
controller_manager:
  ros__parameters:
    update_rate: 50  # Hz

diff_drive_controller:
  ros__parameters:
    type: diff_drive_controller/DiffDriveController
    left_wheel_names: ["left_wheel_joint"]
    right_wheel_names: ["right_wheel_joint"]
    wheel_separation: 0.1125
    wheel_radius: 0.0325
    publish_rate: 50.0
    odom_frame_id: odom
    base_frame_id: base_link
    enable_odom_tf: true
```

This is the production-grade way to do what Isaac Sim's `DifferentialController` + `ArticulationController` OmniGraph nodes do in simulation.

---

## TF2 (Transform Frames)

In simulation, we hardcoded frame IDs (`base_link`, `odom`, `lidar`). On a physical robot, the **TF2** transform tree is critical:

```
map
 └── odom                    (from AMCL or wheel odometry)
      └── base_link          (robot centre)
           ├── lidar         (lidar mount position/orientation)
           ├── left_wheel    (wheel joint)
           └── right_wheel   (wheel joint)
```

The `robot_state_publisher` node reads the robot's URDF and broadcasts static transforms (e.g., lidar mounted 10 cm above base_link, rotated 0°). Dynamic transforms (odom → base_link) come from odometry.

**Why this matters:** When the obstacle avoidance node reads `/scan`, the scan is in the `lidar` frame. If the lidar is mounted backwards or at an angle, the TF tree tells any consumer how to transform those coordinates into the robot's `base_link` frame. In simulation, we set `PublishLaserScan.inputs:frameId = "lidar"` — the same convention.

---

## Complete Physical Launch File

A ROS2 launch file that brings up a physical Jetbot with RPLidar and our obstacle avoidance:

```python
# jetbot_bringup.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Lidar driver
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'lidar',
                'angle_compensate': True,
                'scan_mode': 'Standard',
            }],
        ),

        # Motor driver (custom node)
        Node(
            package='jetbot_driver',
            executable='jetbot_driver_node',
            name='motor_driver',
            parameters=[{
                'wheel_base': 0.1125,
                'max_speed': 0.5,
                'i2c_address': 0x60,
            }],
        ),

        # Odometry from wheel encoders
        Node(
            package='jetbot_driver',
            executable='odometry_node',
            name='odometry',
            parameters=[{
                'wheel_base': 0.1125,
                'wheel_radius': 0.0325,
                'ticks_per_revolution': 2048,
            }],
        ),

        # Robot description (URDF → TF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': open('jetbot.urdf').read()}],
        ),

        # Our obstacle avoidance — IDENTICAL CODE from simulation
        Node(
            package='obstacle_avoidance',
            executable='obstacle_avoidance_node',
            name='obstacle_avoidance',
            parameters=[{
                'linear_speed': 0.2,      # slower for physical safety
                'max_angular_speed': 0.6,
                'ray_range': 3.0,
                'control_hz': 10.0,
            }],
        ),
    ])
```

The obstacle_avoidance_node is literally the same Python file. Only the parameters are adjusted for the physical robot's speed limits.

---

## Sim-to-Real Checklist

| Step | Simulation | Physical | Action Needed |
|------|-----------|----------|---------------|
| 1. Lidar data | OmniGraph `PublishLaserScan` | `rplidar_ros` or equivalent | Install driver, match topic name |
| 2. Motor commands | OmniGraph `SubscribeTwist` → `DiffController` | `ros2_control` or custom driver | Write/install motor driver |
| 3. Odometry | OmniGraph `ComputeOdometry` (ground truth) | Wheel encoders + IMU | Install odometry node, expect drift |
| 4. Frame IDs | Hardcoded `base_link`, `lidar` | URDF + `robot_state_publisher` | Create URDF matching physical geometry |
| 5. Parameters | `linear_speed=0.3`, `ray_range=3.0` | Adjust for hardware limits | Tune via `--ros-args -p key:=value` |
| 6. Network | Docker `--net=host` + FastDDS UDP | WiFi/Ethernet + DDS multicast | Set `ROS_DOMAIN_ID`, ensure multicast works |
| 7. Safety | No consequences of collision | Real damage possible | Add emergency stop, reduce speeds, test slowly |
| 8. Noise handling | Perfect sensor data | Noisy, missing readings | Already handled: code filters inf/NaN |
| 9. Clock | `/clock` from sim (faster/slower than real time) | Wall clock (`use_sim_time: false`) | Don't depend on `/clock` in control loops |
| 10. Latency | ~0 (same process) | 1–10 ms (WiFi) or ~0 (on-robot) | Keep control loops on-robot if latency-sensitive |

---

## Safety Considerations for Physical Robots

Aspects that don't exist in simulation but are critical in the physical world:

1. **Emergency stop:** Always have a hardware kill switch (relay in power line). The ROS2 convention is a `/emergency_stop` topic or a physical big red button.
2. **Watchdog timer:** If `/cmd_vel` stops arriving (node crash, network drop), the motor driver must stop within 100–500 ms. Most drivers implement a command timeout.
3. **Speed limits:** Start at 10% of simulation speed and increase gradually. A 0.3 m/s Jetbot has real momentum.
4. **Sensor failure:** If the lidar USB disconnects, `/scan` stops publishing. The obstacle avoidance node's `if not self.ranges: stop` logic handles this correctly — the robot stops.
5. **Battery monitoring:** Publish battery voltage on `/battery_state` (sensor_msgs/BatteryState). Low voltage causes erratic motor behaviour before complete shutdown.
6. **Bumper sensors:** Physical robots benefit from contact bumpers as a last-resort collision detector, published on `/bumper` (sensor_msgs/BumperEvent in TurtleBot style).
