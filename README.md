# Deterministic Sensor Fusion for Robot Stationary State Detection

## Project Overview

This repository implements a **robust sensor fusion system** for a mobile robot that determines whether the robot is stationary or in motion by fusing data from two asynchronous sensors:

1. **Odometry Sensor** (kinematic data) — provides velocity magnitude at ~30 Hz
2. **Range Sensor** (spatial data) — provides distance measurements at ~20 Hz

The fusion algorithm runs deterministically at a fixed rate (25 Hz) and publishes boolean status (`true` = stationary, `false` = moving) on the `/robot_state/is_stationary` topic.

**Target Application**: Autonomous pool cleaning robot

---

## System Architecture

### High-Level Data Flow

```
┌─────────────────────┐
│  Range Sensor (20Hz)│
│    (simulated)      │
└──────────┬──────────┘
           │
           ├─→ [Ring Buffer]
           │    (spatial history)
           │
┌──────────────────────┐     ┌──────────────────────┐
│ Odometry Sensor(30Hz)├────→│  Fusion Algorithm    │
│    (simulated)       │     │                      │
└──────────────────────┘     │ (Deterministic Loop) │
                             │   @ 25 Hz            │
                    ┌────────→ Decision Logic       │
                    │        │                      │
           [Ring Buffer]     │ (multi-modal fusion) │
            (kinematic       └──────────┬───────────┘
             history)                   │
                                        │
                                   (Boolean)
                                        │
                              ┌─────────┴──────┐
                              │                │
                         Stationary      Moving
                              │                │
                              └────────┬───────┘
                                       │
                              Topic: /robot_state/
                                  is_stationary
```

### Node Architecture

#### 1. **Range Sensor Publisher** (`range_publisher_demo`)
- **File**: `src/range_publisher_demo.cpp`
- **Topics**: 
  - Publishes: `/peak_detection/channel_0/range` (sensor_msgs::Range)
- **Simulation**: Reads CSV telemetry data (`ekf.csv`) at configurable frequency (~20 Hz)
- **Data Source**: Real acoustic range measurements from EKF dataset

#### 2. **Odometry Sensor Publisher** (`odometer_publisher_demo`)
- **File**: `src/odometer_publisher_demo.cpp`
- **Topics**:
  - Publishes: `/odometry/filtered` (nav_msgs::Odometry)
- **Simulation**: Generates odometry data from CSV at ~30 Hz
- **Velocity Extraction**: Computes magnitude from 3D velocity components (vx, vy, vz)

#### 3. **Sensor Fusion Node** (`stationary_detector_publisher`)
- **File**: `src/stationary_detector_publisher.cpp`
- **Core Logic**: `src/static_stationary_detector.cpp` (implementation) + `.hpp` (API)
- **Topics**:
  - Subscribes: `/peak_detection/channel_0/range`, `/odometry/filtered`
  - Publishes: `/robot_state/is_stationary` (std_msgs::Bool)
- **Update Rate**: Deterministic timer loop @ 25 Hz
- **Decision Output**: Boolean state with timestamp

---

## Sensor Fusion Strategy

### Problem Statement
**Challenge**: Two asynchronous sensor streams with different frequencies and latencies must be combined to make a deterministic decision at a fixed rate without race conditions or data loss.

### Solution: Time-Windowed Multi-Modal Fusion with Ring Buffers

#### Key Design Decisions

**1. Asynchronous Data Ingestion with Deterministic Fusion**
   - **Why**: Sensor callbacks (triggered by message arrivals) operate at unpredictable times.
   - **Solution**: Callbacks perform minimal work (O(1) push to ring buffers only); all heavy computation deferred to the **deterministic fusion loop**.
   - **Benefit**: Decouples sensor arrival rates from decision-making, enabling deterministic worst-case latency.

**2. Ring Buffer (Fixed-Capacity Circular Queue)**
   - **Purpose**: Maintain temporal history of measurements without dynamic allocation
   - **Capacity Sizing**: `capacity = sensor_frequency_Hz × window_duration_sec × 3.0`
     - The 3x multiplier accounts for burst arrivals and timing jitter
   - **Data Structure**: Pre-allocated at node initialization; no runtime memory allocation
   - **Access Pattern**: `get_relative(index)` retrieves oldest→newest chronologically
   - **Time Filtering**: Only measurements within `[current_time - window_duration, current_time]` are considered
   - **Complexity**: O(1) push, O(n) read (where n = buffered samples in window)

```cpp
// Example: 500ms window with 20 Hz range sensor
window_duration = 0.5 sec
expected_range_hz = 20 Hz
capacity = 20 × 0.5 × 3.0 = 30 samples
```

**3. Multi-Modal Fusion Logic (Strict AND)**
   - **Odometry Branch**:
     - Computes: max(velocity_magnitude) over time window
     - Decision: "stationary" if ALL velocity samples < velocity_threshold
     - Fallback: Returns `false` (unsafe) if no fresh odometry data exists
   - **Range Branch**:
     - Computes: variance of distance measurements
     - Decision: "stationary" if variance < range_variance_threshold
     - Motivation: Stationary robot should maintain constant distance to obstacles
     - Minimum Samples: Requires ≥3 valid range samples (statistical confidence)
   - **Final Decision**:
     ```
     is_stationary = (odom_indicates_still) AND (range_indicates_still)
     ```
   - **Safety Semantics**: Defaults to "moving" (conservative) when sensor data is insufficient

**4. Time-Window Filtering**
   - All temporal comparisons use absolute timestamps (ROS clock)
   - Cutoff: `cutoff_time = current_timestamp - window_duration`
   - Measurements older than cutoff are ignored (no removal needed; lazy evaluation)
   - Benefit: Handles sensor jitter and reordered packets gracefully

---

## Handling Asynchronous Sensor Streams

### The Challenge
```
Odometry @ 30 Hz:    |--msg1--|--msg2--|--msg3--|--msg4--|--msg5--|...
                           ↓       ↓       ↓       ↓       ↓

Range @ 20 Hz:       |---msg1---|---msg2---|---msg3---|...
                           ↓               ↓               ↓

Fusion Loop @ 25 Hz:  |--tick1--|--tick2--|--tick3--|--tick4--|...
                      (must decide with whatever data
                       has arrived by each tick)
```

### Solution: Decoupled Callback + Fusion Loop Pattern

**Callback Phase (Triggered at Sensor Message Arrival)**
```cpp
void rangeCallback(const sensor_msgs::msg::Range::SharedPtr msg) {
    // Minimal work: O(1) circular buffer push
    detector_->updateRange(msg->range, timestamp);
}

void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Extract 3D velocity → magnitude
    double vel = sqrt(vx² + vy² + vz²);
    // Minimal work: O(1) circular buffer push
    detector_->updateOdometry(vel, timestamp);
}
```

**Fusion Phase (Triggered at Fixed 25 Hz Timer)**
```cpp
void fusionLoopCallback() {
    // Deterministic decision based on buffered data
    bool is_stationary = detector_->isStationary(current_time);
    
    // Publish result
    auto msg = std_msgs::msg::Bool();
    msg.data = is_stationary;
    status_pub_->publish(msg);
}
```

### Properties Achieved
| Property | Behavior |
|----------|----------|
| **Determinism** | Fusion loop runs at fixed 25 Hz; no jitter |
| **Data Coherency** | All samples in ring buffer are timestamped; no stale data |
| **Robustness** | Missing sensor (e.g., range dropout) → safe fallback ("moving") |
| **No Race Conditions** | Callbacks only write to ring buffer; fusion loop only reads |
| **Latency Bound** | Decision latency ≤ 1/25 Hz = 40 ms + callback delay |
| **No Memory Allocation** | Pre-allocated buffers; zero allocations at runtime |

---

## Algorithmic Design Choices

### 1. **Why Velocity Threshold (Odometry)?**
- **Simple & Direct**: Odometry directly measures locomotion.
- **Fast Response**: Detects motion immediately.
- **Threshold**: 0.02 m/s (conservative; catches slow drift)

### 2. **Why Range Variance (Sensor Payload)?**
- **Indirect Validation**: Assumes stationary robot maintains constant distance to obstacles.
- **Noise Rejection**: Variance naturally filters out transient sensor noise.
- **Complementary to Odometry**: Catches motion that odometry might miss (e.g., drift, dead-reckoning error).
- **Threshold**: 0.0005 m² (tuned for acoustic range sensor noise characteristics)

### 3. **Why Strict AND (Both Must Agree)?**
- **Conservative Safety**: Requires consensus from two independent modalities.
- **Fault Tolerance**: Single sensor failure → defaults to safe state (moving).
- **Example**:
  - Odometry says: "stationary" (wheels not moving)
  - Range says: "moving" (distance changing)
  - **Result**: Declare moving (potential sensor drift or undetected slipping)

### 4. **Why Minimum 3 Range Samples?**
- **Statistical Confidence**: Variance becomes meaningful with at least 3 samples.
- **Avoids False Positives**: Prevents random 2-sample noise from triggering stationary state.

### 5. **Why 500ms Window Duration?**
- **Balance**:
  - Too short (e.g., 100ms) → sensitive to single glitches
  - Too long (e.g., 2s) → sluggish response to state changes
- **Adaptive Tuning**: Parameter configurable at node init (default: 0.5 sec)

### 6. **Why Ring Buffers (not deques or lists)?**
- **Predictable Memory**: Fixed allocation; no fragmentation.
- **Cache Locality**: Contiguous storage (better CPU cache utilization).
- **Real-Time Safe**: O(1) worst-case push; suitable for hard real-time constraints.
- **Embedded-Friendly**: No heap churn; deterministic timing.

---

## Prerequisites

### System Requirements
- **OS**: Linux (Ubuntu 22.04 LTS or similar)
- **ROS 2**: Jazzy or Humble
- **C++ Compiler**: GCC 11+ or Clang 14+
- **CMake**: 3.8+
- **Build Tool**: colcon

### Installation

#### 1. Install ROS 2 (if not already installed)
```bash
# For Ubuntu 22.04 (Jazzy)
curl -sSL https://repo.ros2.org/ros.key | sudo apt-key add -
sudo apt update
sudo apt install ros-jazzy-desktop
```

#### 2. Source ROS 2 Setup
```bash
source /opt/ros/jazzy/setup.bash
```

#### 3. Install Build Dependencies
```bash
sudo apt-get install -y \
  build-essential \
  cmake \
  git \
  ros-jazzy-ament-cmake \
  ros-jazzy-rclcpp \
  ros-jazzy-std-msgs \
  ros-jazzy-sensor-msgs \
  ros-jazzy-nav-msgs \
  ros-jazzy-launch \
  python3-colcon-common-extensions
```

---

## Setup Instructions

### 1. Clone the Repository
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <repository-url> .
```

### 2. Verify Directory Structure
```bash
ls -la ~/ros2_ws/src/
# Expected output:
# udemy_ros2_pkg/
# sensors_fusion_test/
# README.md
```

### 3. Initialize ROS 2 Environment
```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
```

---

## Build Instructions

### Full Build
```bash
cd ~/ros2_ws
colcon build
```

### Build Specific Package
```bash
colcon build --packages-select udemy_ros2_pkg --cmake-clean-cache
colcon build --packages-select sensors_fusion_test --cmake-clean-cache
```

### Clean Build (if needed)
```bash
cd ~/ros2_ws
rm -rf build/ install/ log/
colcon build

# Or with cmake-clean-cache
colcon build --packages-select udemy_ros2_pkg --cmake-clean-cache
colcon build --packages-select sensors_fusion_test --cmake-clean-cache
```


### Build Verbosity (Troubleshooting)
```bash
colcon build --event-handlers console_direct+
```

### Expected Output
```
Starting >>> udemy_ros2_pkg
Finished <<< udemy_ros2_pkg [2.45s]          

Starting >>> sensors_fusion_test
Finished <<< sensors_fusion_test [1.23s]

Summary: 2 packages finished [3.68s]
```

---

## Execution Instructions

### 1. Source Built Packages
```bash
cd ~/ros2_ws
source install/setup.bash
```

### 2. Run All Nodes via Launch File
```bash
ros2 launch udemy_ros2_pkg sensors_fusion.launch.py
```

**Expected Output**:
```
[INFO] [launch]: All requested entities launched successfully
[INFO] [range_publisher_demo-1]: Starting range sensor simulator...
[INFO] [odometer_publisher_demo-2]: Starting odometry simulator...
[INFO] [stationary_detector_publisher-3]: Fusion node initialized safely.
```

### 3. Run Individual Nodes

#### Start Range Sensor Publisher
```bash
ros2 run udemy_ros2_pkg range_publisher_demo
```

#### Start Odometry Sensor Publisher (in new terminal)
```bash
ros2 run udemy_ros2_pkg odometer_publisher_demo
```

#### Start Fusion Node (in new terminal)
```bash
ros2 run udemy_ros2_pkg stationary_detector_publisher
```

### 4. Monitor Topics (in new terminal)

#### View Fusion Decision
```bash
ros2 topic echo /robot_state/is_stationary
```

#### View All Topics
```bash
ros2 topic list
```

#### Inspect Topic Content
```bash
# Range messages
ros2 topic echo /peak_detection/channel_0/range

# Odometry messages
ros2 topic echo /odometry/filtered

# Fusion output
ros2 topic echo /robot_state/is_stationary
```

#### Monitor Frequency
```bash
# Check message rates
ros2 topic hz /peak_detection/channel_0/range
ros2 topic hz /odometry/filtered
ros2 topic hz /robot_state/is_stationary
```

### 5. Configuration Parameters

Override parameters at runtime:

```bash
ros2 run udemy_ros2_pkg stationary_detector_publisher \
  -p window_duration_sec:=0.3 \
  -p velocity_threshold:=0.05 \
  -p range_variance_threshold:=0.001 \
  -p fusion_loop_rate_hz:=25.0 \
  -p expected_odom_hz:=30.0 \
  -p expected_range_hz:=20.0
```

### 6. Run Robot Status Subscriber (Optional Debugging)
```bash
ros2 run udemy_ros2_pkg robot_status_subscriber_simulator
```

This prints human-readable status to console:
```
[INFO] Robot state: STATIONARY (true)
[INFO] Robot state: MOVING (false)
...
```

---

## Testing Instructions

### 1. Build Tests
```bash
cd ~/ros2_ws
colcon build --packages-select sensors_fusion_test
```

### 2. Run All Tests
```bash
colcon test --packages-select sensors_fusion_test
```

### 3. View Test Results
```bash
colcon test-result --all --verbose
```

### 4. Run Specific Test
```bash
colcon test --packages-select sensors_fusion_test --ctest-args -R TestNodeCreation
```

### 5. View Test Output
```bash
# Full verbose output
colcon test-result --all --verbose

# Or check XML report directly
cat ~/ros2_ws/build/sensors_fusion_test/test_results/sensors_fusion_test/*.gtest.xml
```

### 6. Expected Test Results
```
TestAcousticSensorSimulator.TestNodeCreation ............ OK
TestAcousticSensorSimulator.TestNodeParamsValidation .... OK
TestAcousticSensorSimulator.TestValidateDataOutput ...... OK
TestAcousticSensorSimulatorB.TestNodeBadParamsInjection1. OK

[==========] 4 tests from 2 test suites ran. (60 ms total)
[  PASSED  ] 4 tests.
```

### Test Coverage

| Test | File | Purpose |
|------|------|---------|
| `TestNodeCreation` | `test/tutorial_test.cpp` | Verify node initialization |
| `TestNodeParamsValidation` | `test/tutorial_test.cpp` | Validate parameter injection |
| `TestValidateDataOutput` | `test/tutorial_test.cpp` | Verify CSV data loading |
| `RangePublisherTest` | `test/range_publisher_test.cpp` | Range sensor simulator |
| `OdometerPublisherTest` | `test/odometer_publisher_test.cpp` | Odometry sensor simulator |

---

## File Structure

```
.
├── README.md                                 # This file
├── udemy_ros2_pkg/
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── src/
│   │   ├── static_stationary_detector.hpp    # Fusion algorithm (API)
│   │   ├── static_stationary_detector.cpp    # Fusion algorithm (implementation)
│   │   ├── stationary_detector_publisher.hpp # Fusion node (API)
│   │   ├── stationary_detector_publisher.cpp # Fusion node (implementation)
│   │   ├── stationary_detector_publisher_main.cpp
│   │   ├── range_publisher_demo.hpp          # Range sensor simulator (API)
│   │   ├── range_publisher_demo.cpp          # Range sensor simulator (impl)
│   │   ├── range_publisher_demo_main.cpp
│   │   ├── odometer_publisher_demo.hpp       # Odometry simulator (API)
│   │   ├── odometer_publisher_demo.cpp       # Odometry simulator (impl)
│   │   ├── odometer_publisher_demo_main.cpp
│   │   ├── robot_status_subscriber_simulator.cpp  # Debug subscriber
│   │   ├── ekf.csv                           # Range/odometry telemetry data
│   │   ├── ekf_test.csv                      # Test dataset
│   │   └── csv_telemetry_reader.hpp          # CSV parsing utility
│   ├── launch/
│   │   └── sensors_fusion.launch.py          # Launch file (all 3 nodes)
│   └── include/
│       └── udemy_ros2_pkg/
│           └── launch/
│
├── sensors_fusion_test/
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── test/
│   │   ├── tutorial_test.cpp                 # Fusion algorithm tests
│   │   ├── range_publisher_test.cpp          # Range sensor tests
│   │   └── odometer_publisher_test.cpp       # Odometry sensor tests
│   └── include/
│       └── sensors_fusion_test/
```
