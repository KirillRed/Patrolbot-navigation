# PatrolBot Navigation

A robust, reactive navigation package for ROS 2 featuring multi-sensor obstacle avoidance with Lidar, Sonar, and Bumper support. Designed for real-world robot patrol and exploration scenarios with a highly modular, configurable architecture.

![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue)
![License](https://img.shields.io/badge/License-MIT-green)
![Python](https://img.shields.io/badge/Python-3.10%2B-yellow)

## Features

- **Multi-Sensor Fusion** — Combines Lidar (primary), Rear Sonar (safety), and Bumpers (emergency stop) for comprehensive obstacle detection
- **Universal Lidar Support** — Handles both Front-0° and Rear-0° sensor configurations automatically  
- **Squeeze Mode** — Intelligent navigation through narrow corridors with smooth centering
- **Simulation & Hardware Support** — Switch between `Twist` and `TwistStamped` messages via a single flag
- **Fully Configurable** — All topics, speeds, and sensor toggles adjustable via launch arguments
- **Graceful Degradation** — Operates with or without optional sensors (Sonar/Bumper)

## Requirements

### Dependencies

| Package | Required | Description |
|---------|----------|-------------|
| `rclpy` | ✅ | ROS 2 Python client library |
| `geometry_msgs` | ✅ | Twist/TwistStamped messages |
| `sensor_msgs` | ✅ | LaserScan and PointCloud2 messages |
| `sensor_msgs_py` | ✅ | PointCloud2 Python utilities |
| `rosaria` | ❌ | Only required for physical bumper support |

### Tested On

- ROS 2 Jazzy
- Ubuntu 24.04
- Python 3.10+

## Installation

### Clone and Build

```bash
# Navigate to your workspace src directory
cd ~/ros2_ws/src

# Clone the repository
git clone https://github.com/KirillRed/Patrolbot-navigation patrolbot_nav

# Build the package
cd ~/ros2_ws
colcon build --packages-select patrolbot_nav

# Source the workspace
source install/setup.bash
```

## Quick Start

### Basic Usage

```bash
# Run with default settings
ros2 launch patrolbot_nav patrolbot.launch.py
```

### Custom Configuration

```bash
# Run with custom speed and topic
ros2 launch patrolbot_nav patrolbot.launch.py speed:=0.5 scan_topic:=/lidar/scan
```

### Simulation Mode (Gazebo)

```bash
# Enable TwistStamped messages for Gazebo
ros2 launch patrolbot_nav patrolbot.launch.py use_sim:=True
```

## Configuration

All parameters can be set via launch arguments:

### Core Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `use_sim` | `False` | Use `TwistStamped` (simulation) instead of `Twist` (hardware) |
| `speed` | `0.25` | Forward cruising speed (m/s) |

### Topic Configuration

| Parameter | Default | Description |
|-----------|---------|-------------|
| `scan_topic` | `/scan` | Lidar LaserScan topic |
| `cmd_vel_topic` | `/cmd_vel` | Velocity command output topic |
| `sonar_topic` | `/sonar_pointcloud2` | Rear sonar PointCloud2 topic |
| `bumper_topic` | `/bumper_state` | Bumper state topic (rosaria) |

### Sensor Toggles

| Parameter | Default | Description |
|-----------|---------|-------------|
| `enable_sonar` | `True` | Enable rear sonar for backup safety |
| `enable_bumper` | `True` | Enable physical bumper failsafe |

### Example: Minimal Lidar-Only Setup

```bash
ros2 launch patrolbot_nav patrolbot.launch.py \
    enable_sonar:=False \
    enable_bumper:=False \
    speed:=0.3
```

### Example: Full Sensor Suite

```bash
ros2 launch patrolbot_nav patrolbot.launch.py \
    scan_topic:=/lidar/scan \
    sonar_topic:=/rear_sonar/points \
    bumper_topic:=/robot/bumpers \
    speed:=0.2
```

## How It Works

### Navigation Priority System

The navigator uses a priority-based decision system for reliable obstacle handling:

```
┌─────────────────────────────────────────────────────────┐
│  Priority 0: BUMPER HIT                                 │
│  → Emergency reverse at -0.15 m/s                       │
├─────────────────────────────────────────────────────────┤
│  Priority 1: EMERGENCY STOP (d_center < 0.5m)           │
│  → Stop and rotate toward open side                     │
├─────────────────────────────────────────────────────────┤
│  Priority 2: SQUEEZE MODE (front corners < 0.6m)        │
│  → Reduced speed (70%), smooth centering between walls  │
├─────────────────────────────────────────────────────────┤
│  Priority 3: TAIL CLEARING (post-squeeze cooldown)      │
│  → Maintain reduced speed, straight heading             │
├─────────────────────────────────────────────────────────┤
│  Priority 4: CRUISE (open space)                        │
│  → Full speed with loose wall-following bias            │
└─────────────────────────────────────────────────────────┘
```

### Sensor Zones

The Lidar scan is divided into sectors for decision-making:

| Zone | Angle Range | Purpose |
|------|-------------|---------|
| Center | ±10° | Emergency stop detection |
| Front-Left | 10° to 35° | Squeeze mode trigger |
| Front-Right | -35° to -10° | Squeeze mode trigger |
| Side-Left | 35° to 75° | Wall-following & direction choice |
| Side-Right | -75° to -35° | Wall-following & direction choice |

### Sonar Integration

When enabled, the rear sonar (PointCloud2) monitors the area behind the robot:

- Detects obstacles within **0.6m** behind the robot (`x < -0.1`)
- Prevents dangerous backward maneuvers when rear is blocked
- Falls back to spin-in-place when trapped front and back

### Bumper Failsafe

If `rosaria` is available and bumpers are enabled:

- Monitors physical bumper contact
- Triggers immediate reverse on any front bumper hit
- Logs emergency alerts for debugging

> **Note:** The bumper feature requires the `rosaria` package. If not installed, the node gracefully disables bumper logic and logs a warning.

## Topics

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/scan` | `sensor_msgs/LaserScan` | Primary lidar input |
| `/sonar_pointcloud2` | `sensor_msgs/PointCloud2` | Rear sonar (optional) |
| `/bumper_state` | `rosaria/BumperState` | Physical bumpers (optional) |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands (hardware) |
| `/cmd_vel` | `geometry_msgs/TwistStamped` | Velocity commands (simulation) |

## Troubleshooting

### Common Issues

**Robot not moving:**
- Verify Lidar topic is publishing: `ros2 topic echo /scan`
- Check if the topic name matches your sensor: `ros2 topic list | grep -i scan`

**Jerky turning behavior:**
- The angular smoothing factor is tunable in the code (`ANG_SMOOTH_ALPHA = 0.7`)
- Higher values = smoother but slower response

**Bumper warnings on startup:**
- This is normal if `rosaria` is not installed
- The node auto-disables bumper logic and continues with Lidar/Sonar only

**Simulation mode not working:**
- Ensure you're using `use_sim:=True` (capital T)
- Verify your simulator expects `TwistStamped` on the cmd_vel topic

## Package Structure

```
patrolbot_nav/
├── launch/
│   └── patrolbot.launch.py    # Main launch file with all parameters
├── patrolbot_nav/
│   ├── __init__.py
│   └── navigator.py           # Core navigation node
├── resource/
│   └── patrolbot_nav
├── test/                       # pytest test files
├── package.xml                 # ROS 2 package manifest
├── setup.py                    # Python package setup
├── setup.cfg
├── LICENSE                     # MIT License
└── README.md
```

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Author

**Kyrylo Redenskyi**  
kredensk@andrew.cmu.edu

---

*Contributions, issues, and feature requests are welcome!*
