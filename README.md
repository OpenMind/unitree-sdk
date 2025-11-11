# Unitree SDK

A ROS 2 package that provides SLAM (Simultaneous Localization and Mapping) capabilities for the Unitree Go2  and G1 robots using an RPLiDAR sensor, the SLAM Toolbox and the Nav2 stack.

## Overview

This package integrates:
- **RPLiDAR** for laser scanning
- **SLAM Toolbox** for mapping and localization
- **Nav2** for autonomous navigation
- **Transform broadcasting** for robot pose integration
- **RViz visualization** for real-time monitoring

## Features

- **Real-time SLAM**: Simultaneous localization and mapping using SLAM Toolbox
- **RPLiDAR Integration**: Support for RPLiDAR A1/A2/A3 series sensors
- **Navigation**: Integration with Nav2 for autonomous navigation
- **Robot Control**: Direct integration with Unitree Go2 movement commands
- **Visualization**: Pre-configured RViz setup for monitoring
- **Transform Management**: Automatic handling of coordinate frame transforms
- **Auto Charging**: AprilTag-based visual docking combined with Nav2 navigation
- **3D Slam Map**: Builds a 3D SLAM map to help the robot navigate

## Prerequisites

- ROS 2 Humble
- Python 3.10+
- RPLiDAR connected via USB (typically `/dev/ttyUSB0`)

## Installation

### 0. Hardware setup:

You can place the RPLiDAR at the center of the robot using a 3D-printed mount.

     ![Front View](./models/body.JPG)
     ![Side View](./models/head.JPG)

You can find the 3D model for the mount in the `models` directory of this repository. We'll shortly be releasing the BOM and details on DIY for it. Stay tuned!

### 1. Clone this repository into your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/OpenmindAGI/unitree_sdk.git
```

### 2. Install dependencies:

```bash
cd ~/ros2_ws
rosdep install --from-paths . --ignore-src -r -y
```

### 3. Install Python dependencies:
```bash
pip install -r requirements.txt
```

### 4. Build the packages:

```bash
colcon build --packages-select unitree_api unitree_go go2_sdk
```

### 5. Source the workspace:

```bash
source install/setup.bash
```

### 6. Set up RPLiDAR permissions:

```bash
# Option 1: Temporary permission (needs to be run each time)
sudo chmod 777 /dev/ttyUSB0

# Option 2: Add user to dialout group (permanent, requires logout/login)
sudo usermod -a -G dialout $USER

# Option 3: Create udev rule (permanent)
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", GROUP="dialout", MODE="0666"' | sudo tee /etc/udev/rules.d/99-rplidar.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## Usage

### Launch SLAM System

To start the complete SLAM system:

```bash
ros2 launch go2_sdk slam_launch.py
```

Once you have done the mapping, you can save the map through the RViz2 interface by clicking on the `Save Map` button and the `Serialize Map` button in the `SlamToolboxPlugin` panel.

### Launch Navigation System
To start the navigation system with SLAM:

```bash
ros2 launch go2_sdk nav2_launch.py map_yaml_file:=<path_to_your_map_yaml_file>
```

### Launch the sensor system
To start the sensor system (RPLiDAR and Unitree Go2 sensors):

```bash
ros2 launch go2_sdk sensor_launch.py
```

### Control the Robot

Once SLAM is running, you can control the robot using:

```bash
# Using keyboard teleop (install first: sudo apt install ros-humble-teleop-twist-keyboard)
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Or publish velocity commands directly
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

Or use an Xbox controller to control the robot

> [!NOTE]
> For the Xbox controller, the RB button is the enable/disable button. Different Xbox controllers may have different button mappings, so you may need to adjust the launch files accordingly.

### REST API

The Go2 SDK provides a REST API for remote control and monitoring of the robot. The API server runs on port 5000 and provides the following endpoints:

#### API Endpoints

**GET /api/status**
- Returns the API status
- Response: `{"status": "OK", "message": "Go2 API is running"}`

**GET /api/pose**
- Returns the current robot pose with covariance
- Response:
```json
{
  "position": {"x": 0.0, "y": 0.0, "z": 0.0},
  "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
  "covariance": [...]
}
```

**POST /api/move_to_pose**
- Sends the robot to a specific pose
- Request body:
```json
{
  "position": {"x": 1.0, "y": 2.0, "z": 0.0},
  "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
}
```
- Response: `{"status": "success", "message": "Moving to specified pose"}`

**GET /api/amcl_variance**
- Returns AMCL localization uncertainty
- Response:
```json
{
  "x_uncertainty": 0.1,
  "y_uncertainty": 0.1,
  "yaw_uncertainty": 5.0
}
```

**GET /api/nav2_status**
- Returns the status of active navigation goals
- Response:
```json
{
  "nav2_status": [
    {
      "goal_id": "abc123...",
      "status": "EXECUTING",
      "timestamp": {"sec": 1234567890, "nanosec": 123456789}
    }
  ]
}
```

**GET /api/map**
- Returns the current occupancy grid map
- Response:
```json
{
  "map_metadata": {
    "map_load_time": {"sec": 1234567890, "nanosec": 123456789},
    "resolution": 0.05,
    "width": 384,
    "height": 384,
    "origin": {
      "position": {"x": -10.0, "y": -10.0, "z": 0.0},
      "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    }
  },
  "data": [0, 0, 0, ...]
}
```

### Visualize in RViz

Launch RViz with the provided configuration:

```bash
rviz2 -d config/rviz.rviz
```

## Run Zenoh Ros2 Bridge
To run the Zenoh bridge for the Unitree Go2, you need to have the Zenoh ROS 2 bridge installed. You can find the installation instructions in the [Zenoh ROS 2 Bridge documentation](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds)

After installing the Zenoh ROS 2 bridge, you can run it with the following command:

```bash
zenoh-bridge-ros2dds -c ./zenoh/zenoh_bridge_config.json5
```

## Production Guidance

We release the Docker image `openmindagi/unitree_sdk`, which provides the full ROS2 system for running the Unitree Go2 SDK.

### Starting the System

To start all services, run the following commands:

```bash
docker-compose up orchestrator -d --no-build
docker-compose up om1_sensor -d --no-build
docker-compose up watchdog -d --no-build
```

This will bring up the entire system.
After the first setup, the system will automatically boot up whenever you start the robot.

### Components Overview
#### 1. watchdog

- If any topics or sensors stop publishing data, the watchdog automatically restarts **om1_sensor**.
- Ensures system stability during long-running sessions.

#### 2. om1_sensor

The **om1_sensor** container manages all low-level sensor drivers:

- **Intel RealSense D435** (depth camera)
- **RPLidar** (LiDAR scanning)

It publishes ROS2 topics consumed by the rest of the system:

- `/om/paths` — Processed path and localization data
- `/scan` — Raw LiDAR scan data

#### **3. orchestrator**

The **orchestrator** service provides an API endpoint and cloud service integration.

It manages:

- SLAM (Simultaneous Localization and Mapping)
- Navigation (**Nav2**)
- Map storage and loading

You can interact with it via REST APIs:

### **API Usage**

#### **Start SLAM**

```bash
curl --location 'http://localhost:5000/start/slam' \
     --header 'Content-Type: application/json' \
     --data '{}'
```

#### **Stop SLAM**

```bash
curl --location 'http://localhost:5000/stop/slam' \
     --header 'Content-Type: application/json' \
     --data '{}'
```

#### **Save a Map**

```bash
curl --location 'http://localhost:5000/maps/save' \
     --header 'Content-Type: application/json' \
     --data '{"map_name": "office"}'
```

#### **Start Navigation (Nav2)**

```bash
curl --location 'http://localhost:5000/start/nav2' \
     --header 'Content-Type: application/json' \
     --data '{"map_name": "maps/office/office.yaml"}'
```

#### **Stop Navigation (Nav2)**

```bash
curl --location 'http://localhost:5000/stop/nav2' \
     --header 'Content-Type: application/json' \
     --data '{}'
```

## Development Guidance

This section provides guidance for developers working on the new OM brainpack system.

![brainpack](./models/backpack.JPG)

You can connect your Linux machine to one of the Ethernet ports of the OM brainpack.

Open the network settings and find the network interface that the robot is connected to. In IPv4 settings, change the IPv4 mode to manual, set the address to 192.168.123.100, and set the mask to 255.255.255.0. After completion, click apply and wait for the network to reconnect.

Then you can subscribe to the topics published by the Orin AGX and the Unitree Go2 robot.


## Troubleshooting

### RPLiDAR Connection Issues

1. Check USB connection:
```bash
ls -la /dev/ttyUSB*
```

2. Verify permissions:
```bash
sudo chmod 777 /dev/ttyUSB0
```

3. Add user to dialout group:
```bash
sudo usermod -a -G dialout $USER
```

### SLAM Performance Issues

- Reduce `scan_buffer_size` if experiencing dropped messages
- Adjust `correlation_search_space_dimension` for better loop closure
- Modify `loop_search_maximum_distance` based on environment size

### Transform Issues

- Ensure all required transforms are being published
- Check TF tree with: `ros2 run tf2_tools view_frames`
- Verify timing with: `ros2 topic echo /tf`

### Robot Control Issues

- Verify cmd_vel messages are being published: `ros2 topic echo /cmd_vel`
- Ensure the robot is in the correct mode for receiving movement commands

### Timestamp Issues

The ROS topic timestamps from the Unitree Go2 are 12 seconds behind the current system time. Please disable your computer’s `Automatic Date & Time` setting and manually sync the timestamp using:

```
sudo date -s "@unix"
```

You can find the timestamp from the Unitree Go2 by running:

```
ros2 topic echo /utlidar/robot_pose --field header.stamp
```

If the Linux machine has an internet connection, you can share the inet connection with the Unitree Go2 robot to automatically sync the time.

```bash
sudo nmcli connection add type ethernet ifname enp112s0 con-name ethernet-shared
sudo nmcli connection modify ethernet-shared ipv4.method shared
sudo nmcli connection modify ethernet-shared ipv4.addresses 192.168.123.1/24
sudo nmcli connection up ethernet-shared
```

and you can verify the shared connection with:

```bash
nmcli connection show ethernet-shared
```