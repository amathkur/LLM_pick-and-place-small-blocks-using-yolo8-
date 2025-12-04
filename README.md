# LLM-Powered Pick-and-Place Robot with YOLOv8 Detection

A ROS2-based intelligent robotic pick-and-place system that combines YOLOv8 object detection, Large Language Model (LLM) natural language processing, and motion planning to enable intuitive voice and text-controlled manipulation of colored blocks and shapes.

![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)
![Python 3.10+](https://img.shields.io/badge/Python-3.10+-green)
![YOLOv8](https://img.shields.io/badge/YOLO-v8-red)
![MoveIt2](https://img.shields.io/badge/MoveIt-2-orange)

## 🎯 Overview

This project enables a Dobot Magician robot arm to:
- **Detect objects** using YOLOv8 trained on 120+ classes of colored shapes
- **Understand natural language commands** via Gemini or Claude LLM
- **Execute pick-and-place tasks** through MoveIt2 motion planning
- **Accept voice or text input** for intuitive robot control
- **Visualize operations** in RViz with real-time feedback

## 🏗️ System Architecture

```
┌─────────────────┐     ┌──────────────┐     ┌─────────────┐
│   Voice/Text    │────▶│     LLM      │────▶│   MoveIt    │
│     Input       │     │  Controller  │     │  Planning   │
└─────────────────┘     └──────────────┘     └─────────────┘
                               │                    │
                               ▼                    ▼
                        ┌──────────────┐     ┌─────────────┐
                        │     YOLO     │     │    RViz     │
                        │  Detection   │────▶│ Visualization│
                        └──────────────┘     └─────────────┘
                               ▲
                               │
                        ┌──────────────┐
                        │   Camera     │
                        │   (USB Cam)  │
                        └──────────────┘
```

## ✨ Features

- **Object Detection**: YOLOv8 trained model detecting 120 object classes (shapes × colors × sizes)
- **Natural Language Control**: Gemini/Claude LLM integration for understanding commands
- **Voice Commands**: Speech recognition for hands-free operation
- **Text Commands**: Desktop GUI for text-based control
- **Motion Planning**: MoveIt2 for collision-free path planning
- **3D Visualization**: Real-time robot state and detection visualization in RViz
- **Camera Offset Compensation**: Automatic 50mm offset adjustment between camera and suction cup
- **Multiple End Effectors**: Support for gripper and suction cup

## 📋 Prerequisites

- **Ubuntu 22.04**
- **ROS2 Humble** (full desktop installation)
- **Python 3.10+**
- **USB Camera** (for object detection)
- **Dobot Magician** (for hardware deployment, optional for simulation)

## 🚀 Installation

### 1. Install ROS2 Humble

Follow the [official ROS2 Humble installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

### 2. Install System Dependencies

```bash
sudo apt update
sudo apt install python3-pip python3-tk ros-humble-moveit ros-humble-rviz2 ros-humble-rosbridge-server
```

### 3. Clone the Repository

```bash
cd ~/
git clone https://github.com/amathkur/LLM_pick-and-place-small-blocks-using-yolo8-.git
cd LLM_pick-and-place-small-blocks-using-yolo8-
```

### 4. Install Python Dependencies

```bash
pip install pymoveit2 google-generativeai opencv-python ultralytics
pip install SpeechRecognition pyaudio  # For voice control
```

### 5. Build the Workspace

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### 6. Configure LLM API

#### For Gemini (Recommended)
1. Get API key from [Google AI Studio](https://aistudio.google.com/)
2. Set the API key as an environment variable:
   ```bash
   export GEMINI_API_KEY='your-gemini-api-key'
   ```
   Or edit `cloud_llm_controller.py` to set your API key directly.

#### For Claude (Alternative)
1. Install: `pip install anthropic`
2. Get API key from [Anthropic](https://www.anthropic.com/)
3. Set the API key as an environment variable:
   ```bash
   export ANTHROPIC_API_KEY='your-anthropic-api-key'
   ```

## 📖 Usage

### Quick Start

**Terminal 1** - Launch RViz, MoveIt, Camera, and YOLO:
```bash
cd ~/LLM_pick-and-place-small-blocks-using-yolo8-
source install/setup.bash
ros2 launch launch/moveit_demo.launch.py
```

**Terminal 2** - Start the LLM Controller:
```bash
cd ~/LLM_pick-and-place-small-blocks-using-yolo8-
source install/setup.bash
python3 cloud_llm_controller.py
```

### Command Examples

#### Basic Movement Commands
```
move left          # Robot shifts left
move right         # Robot shifts right  
move up            # Robot lifts up
move down          # Robot lowers down
move forward       # Robot extends forward
move back          # Robot retracts back
go home            # Return to home position
```

#### Pick and Place Commands
```
pick red cube                              # Pick red cube
pick small red cube and place in left box # Complete pick-place sequence
pick large blue sphere and place right    # Pick and place to right box
stack small red on large blue             # Stack objects
```

#### Natural Language Commands
```
"Hey robot, can you grab that red cube and put it in box A?"
"Please move the sphere to box B"
"Put all cubes in the bin"
```

### Control Modes

1. **Text Mode**: Type commands in the GUI
2. **Voice Mode**: Speak commands using microphone

## 🎨 Supported Object Classes

The YOLO model is trained to detect 120 object classes:

| Sizes | Colors | Shapes |
|-------|--------|--------|
| Small | Red, Green, Blue, Yellow | Cube, Sphere, Cylinder, Pyramid |
| Medium | Orange, Purple, Pink, White | Cube, Sphere, Cylinder, Pyramid |
| Large | Black, Cyan | Cube, Sphere, Cylinder, Pyramid |

## 📁 Project Structure

```
LLM_pick-and-place-small-blocks-using-yolo8-/
├── launch/                    # ROS2 launch files
│   └── moveit_demo.launch.py  # Main launch file
├── urdf/                      # Robot description files
├── meshes/                    # 3D mesh files
├── rviz/                      # RViz configuration
├── cloud_llm_controller.py    # LLM-powered command controller
├── yolo_detection_node.py     # YOLOv8 object detection node
├── camera_publisher.py        # USB camera publisher
├── view_camera.py             # Camera testing utility
├── integrated_pick_place_system.py  # Complete pick-place system
├── dobot_serial_interface.py  # Serial interface for real robot
├── dobot_can_interface.py     # CAN interface for real robot
├── dataset.yaml               # YOLO training dataset config
├── INSTALL.md                 # Detailed installation guide
├── USAGE_README.md            # Usage documentation
├── INTEGRATED_SYSTEM_GUIDE.md # Complete system guide
└── PLANNING_GUIDE.md          # Motion planning documentation
```

## 🔧 Configuration

### Camera Configuration
Edit `camera_publisher.py`:
```python
self.declare_parameter('camera_index', 2)  # Change camera index
```

### YOLO Model
Edit `yolo_detection_node.py`:
```python
model_paths = [
    'path/to/your/model.pt',  # Add your trained model
]
```

### Robot Offset Compensation
The system automatically compensates for the offset between camera and suction cup:
- Suction cup: 55mm from wrist pivot (wrist_pitch)
- Camera: 105mm from wrist pivot (wrist_pitch)
- **Offset**: 50mm (camera is 50mm ahead of suction cup along the arm)

When an object is detected by the camera, the system subtracts 50mm from the X position to ensure the suction cup (not the camera) reaches the object.

## 🧪 Testing

### Check System Status
```bash
ros2 node list              # List active nodes
ros2 topic list             # List available topics
ros2 topic echo /yolo/detections  # View detections
ros2 topic echo /joint_states     # Check robot state
```

### Verify Camera
```bash
python3 view_camera.py      # View camera feed
```

## 🐛 Troubleshooting

| Issue | Solution |
|-------|----------|
| No camera feed | Check USB connection, verify camera index |
| YOLO not detecting | Check lighting, verify model loaded |
| Robot not moving | Verify MoveIt is running, try "go home" |
| Voice not working | Check microphone permissions, install pyaudio |
| LLM not responding | Verify API key and internet connection |

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## 📚 Additional Documentation

- [Installation Guide](INSTALL.md)
- [Usage Documentation](USAGE_README.md)
- [Integrated System Guide](INTEGRATED_SYSTEM_GUIDE.md)
- [Planning Guide](PLANNING_GUIDE.md)
- [Pick-Place Guide](PICK_PLACE_GUIDE.txt)

## 📄 License

This project is open source. See the repository for license details.

## 🙏 Acknowledgments

- [ROS2](https://docs.ros.org/) - Robot Operating System
- [MoveIt2](https://moveit.ros.org/) - Motion Planning Framework
- [Ultralytics YOLOv8](https://github.com/ultralytics/ultralytics) - Object Detection
- [Google Gemini](https://ai.google.dev/) - Large Language Model
- [Dobot](https://www.dobot.cc/) - Robot Hardware
