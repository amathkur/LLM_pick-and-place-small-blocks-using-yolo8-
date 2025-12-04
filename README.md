# DoBot LLM Pick-and-Place System with YOLOv8 Detection

A complete robotic pick-and-place system combining **YOLOv8 object detection**, **Gemini LLM natural language control**, and **ROS2 MoveIt motion planning** for the DoBot Magician Lite robotic arm.

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![Python](https://img.shields.io/badge/Python-3.10+-green)
![License](https://img.shields.io/badge/License-MIT-yellow)

## ✨ Features

- **🎯 YOLOv8 Object Detection** - Detects 120 object classes (4 shapes × 3 sizes × 10 colors) with real-time 3D positioning
- **🤖 Gemini LLM Control** - Natural language command interpretation for intuitive robot control
- **🗣️ Voice & Text Commands** - Speak or type commands to control the robot
- **📐 ROS2 MoveIt Integration** - Collision-free motion planning with RViz visualization
- **🔄 Camera Offset Compensation** - Automatic 50mm camera-to-suction offset correction
- **📚 Demonstration Learning** - Train policies through manual demonstrations
- **🏠 Automatic Home Return** - Robot returns to home position after tasks

## 📋 System Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                           User Interface                                 │
│  ┌─────────────┐     ┌─────────────┐     ┌─────────────┐               │
│  │   Voice     │     │    Text     │     │   GUI       │               │
│  │  Commands   │     │  Commands   │     │  Controls   │               │
│  └──────┬──────┘     └──────┬──────┘     └──────┬──────┘               │
└─────────┼───────────────────┼───────────────────┼───────────────────────┘
          │                   │                   │
          └───────────────────┼───────────────────┘
                              ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                      Gemini LLM Commander                               │
│  ┌────────────────────────────────────────────────────────────────┐    │
│  │  Natural Language Processing → JSON Robot Commands             │    │
│  │  "pick red cube" → {"action": "pick_object", "color": "red"}   │    │
│  └────────────────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────────────────┘
                              │
          ┌───────────────────┼───────────────────┐
          ▼                   ▼                   ▼
┌──────────────────┐ ┌──────────────────┐ ┌──────────────────┐
│  YOLOv8 Vision   │ │  MoveIt Motion   │ │  Robot Control   │
│  ┌────────────┐  │ │  ┌────────────┐  │ │  ┌────────────┐  │
│  │ Detection  │  │ │  │ Planning   │  │ │  │ Execution  │  │
│  │ + Color    │  │ │  │ + IK       │  │ │  │ + Gripper  │  │
│  │ + Size     │  │ │  │ + Collision│  │ │  │ + Suction  │  │
│  └────────────┘  │ │  └────────────┘  │ │  └────────────┘  │
└──────────────────┘ └──────────────────┘ └──────────────────┘
          │                   │                   │
          └───────────────────┴───────────────────┘
                              ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                        DoBot Magician Lite                              │
└─────────────────────────────────────────────────────────────────────────┘
```

## 🚀 Quick Start

### Prerequisites

- Ubuntu 22.04
- ROS2 Humble
- Python 3.10+
- USB Camera
- DoBot Magician Lite (optional for simulation)

### Installation

```bash
# 1. Install ROS2 Humble
sudo apt update
sudo apt install python3-pip python3-tk ros-humble-moveit ros-humble-rviz2 ros-humble-rosbridge-server

# 2. Install Python dependencies
pip install pymoveit2 google-generativeai opencv-python ultralytics SpeechRecognition pyaudio

# 3. Clone and build
git clone <this-repo-url>
cd dobot_rviz_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash

# 4. Configure Gemini API (see Configuration section)
```

### Launch the System

```bash
# Terminal 1: Start RViz + MoveIt + Camera + YOLO
cd ~/dobot_rviz_ws
source install/setup.bash
ros2 launch launch/moveit_demo.launch.py

# Terminal 2: Start LLM Controller with GUI
python3 cloud_llm_controller.py
```

## 🎮 Usage

### Text Commands

Type commands directly in the GUI:

```
move left 3 cm          # Move robot left by 3 centimeters
go up 2 cm              # Move robot up by 2 centimeters
pick red cube           # Pick up a red cube detected by YOLO
pick small blue sphere  # Pick specific size and color object
place at 0.2 0.0 0.1    # Place object at coordinates (x, y, z)
go home                 # Return to home position
go to ready             # Move to ready position
```

### Voice Commands

Click "Voice Command" button and speak:

- "move the robot left"
- "pick up the red block"
- "go back to home position"
- "place the object in front"

### Natural Language Examples

The Gemini LLM understands various phrasings:

| User Says | Robot Action |
|-----------|--------------|
| "Can you grab that red cube?" | Picks red cube |
| "Move a little to the left" | Moves 3cm left |
| "Put it down over there" | Places object |
| "Take me home" | Returns to home |

## 🤖 Gemini LLM Commander

The system uses Google's Gemini AI to interpret natural language commands and convert them to robot actions.

### Command Flow

```
User Input → Gemini API → JSON Command → Robot Execution
     │            │              │              │
     ▼            ▼              ▼              ▼
  "pick the    Process &    {"action":     Execute
   red cube"   Interpret    "pick_object", pick
                            "color":"red"} sequence
```

### Supported Actions

| Action | Parameters | Example |
|--------|------------|---------|
| `move_to_pose` | x, y, z (meters) | Relative movement |
| `pick_object` | size, color, type | Pick with YOLO detection |
| `place_object` | x, y, z | Place at coordinates |
| `home` | - | All joints to 0° |
| `ready` | - | Ready position (0, 0.5, -1.0, 0) rad |

### Movement Commands

Directional commands are interpreted as relative movements:

| Direction | Axis | Sign |
|-----------|------|------|
| Up/Down | Z | +/- |
| Left/Right | Y | +/- |
| Forward/Back | X | +/- |

### Coordinate Frame Reference

The robot uses the `base_link` frame:
- **+X** → Forward (away from robot)
- **+Y** → Left (from robot's perspective)
- **+Z** → Up

### API Configuration

```python
# In cloud_llm_controller.py
api_key = "YOUR_GEMINI_API_KEY"  # Get from Google AI Studio
api_url = "https://generativelanguage.googleapis.com/v1beta/models/gemini-2.0-flash:generateContent"
```

### Error Handling

- **Rate Limiting (429)**: Automatic retry with exponential backoff (2^attempt seconds)
- **Invalid JSON**: Logged and command rejected
- **Network Errors**: Caught and reported to user

### GUI Features

The Gemini Desktop Commander GUI provides:
- **Text Entry**: Type natural language commands
- **Voice Button**: Click to speak commands
- **End Effector Selection**: Toggle between gripper and suction
- **Status Display**: Real-time command status
- **Response History**: View past commands and responses

## 🎯 Object Detection

### YOLO Class Structure

Objects are classified as: `{size}_{color}_{shape}`

**Sizes**: small, medium, large (3 total)  
**Colors**: red, green, blue, yellow, orange, purple, pink, white, black, cyan (10 total)  
**Shapes**: cube, sphere, cylinder, pyramid (4 total)

### Detection Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/yolo/detection_image` | Image | Annotated camera feed |
| `/yolo/detections` | String | Detection descriptions |
| `/yolo/target_pose` | PoseStamped | 3D object position |
| `/yolo/detection_markers` | MarkerArray | RViz visualization |

### Camera Calibration

The system uses homography calibration for pixel-to-robot coordinate transformation:

```python
# Calibration points (image pixels → robot mm)
Image: [(152,29), (499,49), (481,388), (142,376)]
Robot: [(165,-100), (365,-100), (365,100), (165,100)]
```

## 📐 Camera Offset Compensation

The camera is mounted 50mm ahead of the suction cup. The system automatically compensates:

```
Camera sees object at X=0.200m
         ↓
Offset applied: Suction moves to X=0.150m (0.200 - 0.050)
         ↓
Object centered under suction cup ✓
```

### Physical Layout

```
wrist_pitch → (+35mm Z) → suction_cup → (+25mm X) → camera_link
                                                         ↓ (points down)
```

## 🏠 Preset Positions

| Position | Joint Values (radians) | Description |
|----------|------------------------|-------------|
| Home | [0, 0, 0, 0] | All joints at zero |
| Ready | [0, 0.5, -1.0, 0] | Ready for picking |

## 🛠️ Troubleshooting

### Common Issues

| Issue | Solution |
|-------|----------|
| Robot not moving | Check RViz for "Solution found" message |
| YOLO not detecting | Verify camera feed: `ros2 topic echo /camera/image_raw` |
| Gemini not responding | Check API key and internet connection |
| Voice not working | Install: `pip install SpeechRecognition pyaudio` |
| Planning fails | Target may be out of reach or in collision |

### Verification Commands

```bash
# Check nodes
ros2 node list

# Check topics
ros2 topic list

# View detections
ros2 topic echo /yolo/detections

# Check transforms
ros2 run tf2_ros tf2_echo base_link camera_link

# Verify camera
ros2 topic hz /camera/image_raw
```

## 📚 Detailed Documentation

For more in-depth information, see:

- [Installation Guide](INSTALL.md) - Detailed setup instructions
- [Usage Guide](USAGE_README.md) - Complete usage documentation
- [Integrated System Guide](INTEGRATED_SYSTEM_GUIDE.md) - Pick-place workflow
- [Planning Guide](PLANNING_GUIDE.md) - MoveIt motion planning details
- [Camera Calibration](calibrate_camera_position.md) - Camera setup
- [Demonstration Training](DEMONSTRATION_TRAINING_README.md) - Learning from demos
- [Camera Offset](CAMERA_OFFSET_SUMMARY.md) - Offset compensation details

## 📁 Project Structure

```
dobot_rviz_ws/
├── launch/
│   └── moveit_demo.launch.py      # Main launch file
├── cloud_llm_controller.py        # Gemini LLM controller with GUI
├── voice_text_controller.py       # Voice/text command handler
├── yolo_detection_node.py         # YOLOv8 object detection
├── camera_publisher.py            # Camera feed node
├── integrated_pick_place_system.py # Complete pick-place system
├── dobot_moveit_bridge.py         # MoveIt interface
├── magician_moveit_config/        # MoveIt configuration
│   └── config/
│       ├── kinematics.yaml
│       ├── ompl_planning.yaml
│       └── joint_limits.yaml
├── urdf/                          # Robot description
├── meshes/                        # 3D models
└── rviz/                          # RViz configurations
```

## 🤝 Contributing

Contributions are welcome! Please feel free to submit issues or pull requests.

## 📄 License

This project is licensed under the MIT License.

---

**Happy robot controlling! 🤖🎙️📹**
