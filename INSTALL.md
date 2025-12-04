# DoBot ROS2 LLM Controller with YOLO Object Detection

## Installation Guide

⚠️ **Prerequisite**: You need ROS2 Humble installed locally on your machine OR access over the network to a robot/computer with ROS2 installed. This project connects to ROS2 systems on the DoBot robot, so a running ROS2 environment is required.

Installation includes the following steps:

1. **Install the ROS2 Workspace and Dependencies**
2. **Clone this repository**
3. **Install Python dependencies**
4. **Configure the Gemini API**
5. **Launch the system**

### 1. Install ROS2 Humble and Dependencies

Follow the official ROS2 Humble installation guide for Ubuntu 22.04: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

Install additional packages:
```bash
sudo apt update
sudo apt install python3-pip python3-tk ros-humble-moveit ros-humble-rviz2 ros-humble-rosbridge-server
pip install pymoveit2 google-generativeai opencv-python ultralytics
```

### 2. Clone the Repository
```bash
git clone <this-repo-url>
cd dobot_rviz_ws
```

### 3. Build the Workspace
```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### 4. Configure LLM API
The system supports both Gemini (Google) and Claude (Anthropic) LLMs.

#### Gemini (Current Default)
- Get API key from Google AI Studio
- Edit `cloud_llm_controller.py` and set: `api_key = 'your-gemini-api-key'`
- The code is currently configured for Gemini

#### Claude
- Install: `pip install anthropic`
- Get API key from Anthropic
- Edit `cloud_llm_controller.py` and set: `api_key = 'your-anthropic-api-key'`
- Change import to `import anthropic`

### 5. Launch the System
For simulation with RViz:
```bash
ros2 launch launch/moveit_demo.launch.py
```

In a new terminal:
```bash
python3.10 cloud_llm_controller.py
```

For real robot, ensure the DoBot is connected and modify the launch accordingly.

### YOLO Object Detection
The system uses YOLOv8 to detect objects, including color and size estimation.

- **Color Detection**: Implemented in `yolo_detection_node.py` using OpenCV color space conversion.
- **Size Estimation**: Based on bounding box area and known object sizes.

Example commands in the GUI:
- "pick red cube with suction"
- "place object at 0.2 0.0 0.1"

### Troubleshooting
- If MoveIt planning fails, try adjusting kinematics settings in `launch/moveit_demo.launch.py`.
- Ensure all nodes are running: `ros2 node list`
- Check topics: `ros2 topic list`

For MCP alternative (if MoveIt issues persist), see MCP_README.md for ROS MCP server integration.</content>
<parameter name="filePath">/home/abdulhamid/dobot_rviz_ws/README.md