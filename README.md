# RECLAIM — Autonomous Waste Collection and Sorting Robot

RECLAIM is a 5th-year engineering capstone project building an autonomous indoor robot that navigates post-event venues, detects waste on the ground, picks it up with a 6DOF robotic arm, classifies it (recyclable / compost / landfill), and deposits it in the correct bin.

**Demo date: March 26, 2026**

## Hardware

- **Compute:** Advantech MIC-711 (Jetson Orin NX), JetPack 5.x, Ubuntu 20.04
- **Microcontroller:** Teensy 4.1 (USB serial, micro-ROS)
- **Camera:** OAK-D Lite (DepthAI v3, stereo depth + YOLO on-device)
- **LiDAR:** RPLIDAR A1M8 (360° scan, 0.15-12m range)
- **Drive:** 2x JGB37-520 motors + 2x Cytron MD10C drivers, differential drive
- **Arm:** 6DOF with 6x MG996R servos, powered by 6V buck converter
- **Battery:** ZapLitho 12.8V 22Ah LiFePO4
- **Network:** GL.iNet Mango travel router (SSH from Mac to MIC-711)

## Software Stack

- **ROS2 Humble** via RoboStack/conda (`ros_env` environment)
- **SLAM Toolbox** for mapping and localization
- **Nav2** for path planning and obstacle avoidance
- **micro-ROS** on Teensy 4.1 (motor control, encoders, arm servos)
- **DepthAI v3** for OAK-D Lite camera pipeline
- **YOLOv8n** for waste detection (fine-grained classes mapped to bins)
- **Foxglove Studio** for visualization and debugging

## Repository Structure

```
reclaim_ws/
├── src/
│   ├── reclaim_perception/       # OAK-D pipeline, YOLO inference, detection publisher
│   ├── reclaim_navigation/       # SLAM, Nav2, LiDAR driver configs
│   ├── reclaim_control/          # Teensy firmware (PlatformIO) + micro-ROS config
│   │   └── firmware/
│   │       ├── motor_encoder_test/   # Motor + encoder test firmware
│   │       └── servo_test/           # Interactive 6-servo test firmware
│   ├── reclaim_bringup/          # Top-level launch, state machine, system integration
│   └── reclaim_interfaces/       # Custom ROS2 msg/srv/action definitions
├── tests/                        # Hardware test scripts (camera, lidar, teensy)
├── docs/                         # Project documentation
│   ├── Teensy.md                 # Teensy 4.1 reference (pin map, flash procedure)
│   └── Perception_Strategy.md    # Model training strategy, OAK-D depth pipeline
└── docs/                         # Project documentation
```

## Quick Start (on MIC-711)

```bash
# Activate ROS2 environment
conda activate ros_env

# Build a package
cd ~/reclaim_ws && colcon build --packages-select reclaim_perception

# Launch
source install/setup.bash && ros2 launch reclaim_perception perception.launch.py
```

## Team

Engineering capstone project, Western University, 2025-2026.
