# DoBot Pick-and-Place with YOLOv8 and LLM Control

A complete robotic pick-and-place system integrating **YOLOv8 object detection**, **LLM-powered natural language control**, and **ROS2 MoveIt motion planning** for the DoBot Magician arm.

## 🎯 Features

- **YOLOv8 Object Detection**: Detects 400+ object classes (10 shapes × 2 sizes × 20 colors)
- **LLM Natural Language Control**: Use natural language or voice commands with Gemini/Claude
- **ROS2 MoveIt Integration**: Motion planning with collision avoidance
- **RViz Visualization**: Real-time 3D visualization of robot and detected objects
- **Camera Offset Compensation**: Automatic 50mm offset compensation for accurate picking
- **Demonstration Training**: Learn from human demonstrations
- **Voice & Text Control**: Support for both voice and text-based commands

## 📋 Table of Contents

- [Prerequisites](#-prerequisites)
- [Installation](#-installation)
- [Quick Start](#-quick-start)
- [System Components](#-system-components)
- [Usage](#-usage)
- [Object Detection](#-object-detection)
- [Configuration](#-configuration)
- [Troubleshooting](#-troubleshooting)
- [Detailed Documentation](#-detailed-documentation)

## ⚙️ Prerequisites

- **Ubuntu 22.04** (recommended)
- **ROS2 Humble** installed locally or network access to a ROS2 system
- **Python 3.10+**
- **USB Camera** (tested with camera index 2)

## 📦 Installation

### 1. Install ROS2 Humble and Dependencies

Follow the official [ROS2 Humble installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) for Ubuntu 22.04.

Install additional packages:

```bash
sudo apt update
sudo apt install python3-pip python3-tk ros-humble-moveit ros-humble-rviz2 ros-humble-rosbridge-server
pip install pymoveit2 google-generativeai opencv-python ultralytics
```

### 2. Clone the Repository

```bash
git clone https://github.com/amathkur/LLM_pick-and-place-small-blocks-using-yolo8-.git
cd LLM_pick-and-place-small-blocks-using-yolo8-
```

### 3. Build the Workspace

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### 4. Configure LLM API (Optional)

The system supports both Gemini (Google) and Claude (Anthropic) LLMs.

**Gemini (Default):**
1. Get API key from [Google AI Studio](https://aistudio.google.com/)
2. Edit `cloud_llm_controller.py` and locate the `api_key` variable (typically near the top of the file or in the initialization section):
   ```python
   api_key = 'your-gemini-api-key'
   ```

**Claude:**
```bash
pip install anthropic
```
Edit `cloud_llm_controller.py` with your Anthropic API key.

### 5. Voice Commands (Optional)

```bash
pip install SpeechRecognition pyaudio
sudo apt-get install portaudio19-dev python3-pyaudio
```

## 🚀 Quick Start

### Launch the Complete System

**Terminal 1 - Start RViz + MoveIt + Camera + YOLO:**
```bash
# Navigate to the cloned repository directory
source install/setup.bash
ros2 launch launch/moveit_demo.launch.py
```

**Terminal 2 - Start Controller (wait 10 seconds after Terminal 1):**
```bash
# In the same repository directory
python3 cloud_llm_controller.py
```
Choose mode: `1` for Text or `2` for Voice

### Basic Commands

| Command | Action |
|---------|--------|
| `move left` | Move in +Y direction |
| `move right` | Move in -Y direction |
| `move up` | Move in +Z direction |
| `move down` | Move in -Z direction |
| `move forward` | Move in +X direction |
| `move back` | Move in -X direction |
| `go home` | Return to home position |
| `pick cube` | Pick detected cube |
| `pick red sphere and place in box a` | Full pick-and-place |

### Natural Language Examples

```
"pick small red cube and place in left box"
"can you grab that blue sphere?"
"stack small yellow on large green"
"move the arm to the cylinder"
```

## 🤖 System Components

### System Architecture

```
┌─────────────┐     ┌──────────────┐     ┌─────────────┐
│   Voice/    │────▶│     LLM      │────▶│   MoveIt    │
│    Text     │     │  Controller  │     │  Planning   │
└─────────────┘     └──────────────┘     └─────────────┘
                           │                     │
                           ▼                     ▼
                    ┌──────────────┐     ┌─────────────┐
                    │     YOLO     │     │    RViz     │
                    │  Detection   │────▶│ Visualization│
                    └──────────────┘     └─────────────┘
                           ▲                     
                           │                     
                    ┌──────────────┐            
                    │   USB Camera │            
                    └──────────────┘            
```

### Key Components

| Component | File | Description |
|-----------|------|-------------|
| Camera Publisher | `camera_publisher.py` | USB camera feed (index 2) |
| YOLO Detection | `yolo_detection_node.py` | Object detection with trained model |
| LLM Controller | `cloud_llm_controller.py` | Natural language processing |
| Integrated System | `integrated_pick_place_system.py` | Complete pick-place workflow |
| MoveIt Demo | `launch/moveit_demo.launch.py` | Main launch file |

### Coordinate Frames

```
world (grid/floor at z=0)
  └─ base_footprint (z=0.10m) ← Robot base
      └─ base_link (z=0.01m)
          └─ shoulder → elbow → wrist → wrist_pitch
              └─ suction_cup (55mm from wrist_pitch)
                  └─ camera_link (50mm from suction)
```

## 🔍 Object Detection

### Trained YOLO Model

- **Location**: `dobot_llm_project/models/yolo_shapes_best.pt`
- **Classes**: 400 (10 shapes × 2 sizes × 20 colors)

### Supported Objects

**Shapes (10):**
cube, cylinder, sphere, pyramid, cone, prism, hexagon, star, disk, torus

**Sizes (2):**
small, large

**Colors (20):**
red, green, blue, yellow, orange, purple, pink, cyan, magenta, lime, navy, teal, maroon, olive, brown, coral, turquoise, violet, indigo, gold

### Detection Topics

```bash
# View detections
ros2 topic echo /yolo/detections

# Check detection rate
ros2 topic hz /yolo/detections

# View camera feed
ros2 topic echo /camera/image_raw
```

## ⚙️ Configuration

### Camera Settings

Edit `camera_publisher.py`:
```python
self.declare_parameter('camera_index', 2)  # Change camera index if needed
```

### YOLO Model

Edit `yolo_detection_node.py`:
```python
model_paths = [
    'path/to/your/model.pt',  # Add custom model
]
```

### Workspace Limits

```python
# In the controller
X: 0.15m to 0.35m (forward reach)
Y: -0.15m to 0.15m (left-right)
Z: 0.05m to 0.35m (height above base)
```

### Camera Offset

The system automatically compensates for the 50mm camera offset:
- Camera detects object at position X
- Robot moves to X - 50mm for accurate pickup

## 🔧 Troubleshooting

### Robot Not Moving

1. Check MoveGroup is running: `ros2 node list | grep move_group`
2. Verify joint states: `ros2 topic echo /joint_states --once`
3. Try simple command: `go home`

### No Object Detection

1. Check camera feed: `ros2 topic hz /camera/image_raw`
2. Verify camera connected: `ls /dev/video*`
3. Ensure good lighting
4. View camera: `python3 view_camera.py`

### Planning Fails

- "Solution found but controller failed" - This is normal in simulation (no real robot)
- Check target is within workspace limits
- Reset with `go home` command

### Voice Not Working

1. Test microphone: `arecord -l`
2. Check permissions: `sudo usermod -a -G audio $USER`
3. Use text mode to verify system works

### Verify Transforms

```bash
# Check TF tree
ros2 run tf2_tools view_frames
evince frames.pdf

# Verify camera offset
ros2 run tf2_ros tf2_echo suction_cup camera_link
# Expected: translation X=0.050
```

## 📚 Detailed Documentation

For more detailed information, see these guides:

| Guide | Description |
|-------|-------------|
| [INSTALL.md](INSTALL.md) | Detailed installation instructions |
| [USAGE_README.md](USAGE_README.md) | Usage and commands reference |
| [INTEGRATED_SYSTEM_GUIDE.md](INTEGRATED_SYSTEM_GUIDE.md) | Complete system guide with visual feedback |
| [DEMONSTRATION_TRAINING_README.md](DEMONSTRATION_TRAINING_README.md) | Training from demonstrations |
| [PLANNING_GUIDE.md](PLANNING_GUIDE.md) | MoveIt planning and visualization |
| [CAMERA_OFFSET_SUMMARY.md](CAMERA_OFFSET_SUMMARY.md) | Camera offset compensation details |
| [calibrate_camera_position.md](calibrate_camera_position.md) | Camera calibration instructions |
| [BASE_POSITION_FIX.md](BASE_POSITION_FIX.md) | Robot base position configuration |

### Quick Reference Files

| File | Description |
|------|-------------|
| [OBJECT_DETECTION_COMMANDS.txt](OBJECT_DETECTION_COMMANDS.txt) | YOLO detection command reference |
| [PICK_PLACE_GUIDE.txt](PICK_PLACE_GUIDE.txt) | Pick-and-place quick start guide |

## 🎮 RViz Visual Feedback

When YOLO detects objects, you'll see in RViz:

- 🔴 **Red Spheres** = Detected cubes
- 🟢 **Green Spheres** = Detected spheres
- 🔵 **Blue Spheres** = Detected cylinders
- 🟡 **Yellow Spheres** = Detected pyramids
- 🟣 **Purple Arrows** = Target positions (offset-compensated)

## 📊 Training with Demonstrations

Instead of pure reinforcement learning, train by demonstrating:

1. **Start Recording**: `python3 send_command.py start`
2. **Position Robot**: Use Joint State Publisher GUI
3. **Record Pick**: `python3 send_command.py pick`
4. **Record Place**: `python3 send_command.py place`
5. **Save**: `python3 send_command.py save`
6. **Train**: `python3 send_command.py train`

See [DEMONSTRATION_TRAINING_README.md](DEMONSTRATION_TRAINING_README.md) for details.

## 🏭 Real Robot Deployment

For real DoBot hardware:

1. **Connect Robot**:
   ```bash
   python3 dobot_serial_interface.py  # Serial connection
   # or
   python3 dobot_can_interface.py     # CAN connection
   ```

2. **Verify Connection**:
   ```bash
   ros2 topic list | grep /dobot
   ```

3. **Run System**:
   ```bash
   python3 integrated_pick_place_system.py
   ```

## 📄 License

This project is open source. See LICENSE file for details.

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

---

**System Status**: ✅ Ready for testing  
**YOLO Detection**: ✅ 400 object classes  
**Voice/Text Control**: ✅ Supported  
**Visual Feedback**: ✅ RViz markers enabled

Happy robot controlling! 🤖🎙️📹
