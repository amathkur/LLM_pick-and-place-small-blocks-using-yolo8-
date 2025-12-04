# RViz-Gazebo Control System for Dobot Magician with YOLOv9 Vision

This folder contains the launch files and configuration to run both RViz and Gazebo simultaneously, control the Dobot Magician robot from RViz, and use YOLOv9 computer vision for object detection and autonomous picking.

## Features

- **Dual Visualization**: RViz and Gazebo running simultaneously
- **Joint Control**: Interactive control via joint_state_publisher_gui sliders
- **YOLOv9 Integration**: Real-time object detection using USB camera
- **Dobot Hardware Control**: Direct USB connection to physical Dobot robot
- **Autonomous Picking**: Vision-guided object manipulation

## Files

- `rviz_gazebo_control.launch.py`: Main launch file that starts RViz, Gazebo, and YOLOv9 controller
- `magician_lite_control.urdf`: URDF file with proper mesh paths and Gazebo joint trajectory controller plugin
- `joint_state_to_commands.py`: Python script that converts joint_states to joint trajectory commands
- `yolov9_dobot_controller.py`: YOLOv9 vision and Dobot control integration

## Hardware Requirements

- **Dobot Magician Robot**: Connected via USB
- **USB Camera**: For object detection (tested with standard webcams)
- **Ubuntu 22.04**: With ROS2 Humble
- **NVIDIA GPU**: Recommended for YOLOv9 performance (optional)

## Software Dependencies

- ROS2 Humble
- Gazebo Ignition
- OpenCV
- PyTorch
- YOLOv9 model weights
- pydobot or pydobot2 library

## Installation

1. **Install system dependencies**:
   ```bash
   sudo apt-get update
   sudo apt-get install -y ros-humble-cv-bridge ros-humble-image-transport
   ```

2. **Install Python dependencies**:
   ```bash
   cd /home/abdulhamid/dobot_rviz_ws/src/dobot_rviz_demo
   pip3 install -r requirements.txt
   ```

3. **Download YOLOv9 weights**:
   - Visit: https://github.com/WongKinYiu/yolov9/releases
   - Download a model (e.g., `yolov9-c.pt` for speed or `yolov9-e.pt` for accuracy)
   - Place in a directory and update the path in `yolov9_dobot_controller.py`

4. **Run setup script**:
   ```bash
   chmod +x setup_yolov9.sh
   ./setup_yolov9.sh
   ```

## Usage

### 1. Build the workspace:
```bash
cd /home/abdulhamid/dobot_rviz_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --merge-install --packages-select dobot_rviz_demo magician_description
source install/setup.bash
```

### 3. Test the integration:
```bash
cd /home/abdulhamid/dobot_rviz_ws/src/dobot_rviz_demo
python3 test_integration.py
```

### 4. Calibrate camera (optional but recommended):
```bash
python3 /home/abdulhamid/dobot_rviz_ws/src/dobot_rviz_demo/dobot_rviz_demo/scripts/camera_calibration.py test
```
```bash
ros2 launch /home/abdulhamid/dobot_rviz_ws/src/dobot_rviz_demo/dobot_rviz_demo/launch/rviz_gazebo_control/rviz_gazebo_control.launch.py
```

### 4. Control Modes:

#### Manual Control (RViz/Gazebo):
- Use the joint_state_publisher_gui sliders to manually control joints
- Robot moves in both RViz and Gazebo simultaneously

#### Autonomous Vision Control:
- YOLOv9 detects objects in real-time from USB camera
- When target object (default: "bottle") is detected, robot automatically picks it up
- Camera feed is published to `/camera/image_raw` topic

## Configuration

### YOLOv9 Settings (in `yolov9_dobot_controller.py`):
- `target_class`: Object class to detect and pick up
- `camera_index`: USB camera device index
- `workspace_bounds`: Safe workspace limits for robot movement
- `model_path`: Path to YOLOv9 weights file

### Robot Settings:
- `home_position`: Safe home position for robot
- Serial port auto-detection for Dobot connection

## Topics

- `/joint_states`: Joint positions from GUI/manual control
- `/joint_trajectory`: Trajectory commands for Gazebo
- `/camera/image_raw`: Camera feed with YOLOv9 detections
- `/robot_description`: Robot URDF for visualization

## Troubleshooting

### Hardware Connection Issues:
```bash
# Check serial ports
ls /dev/ttyACM* /dev/ttyUSB*

# Check camera devices
ls /dev/video*

# Check USB devices
lsusb | grep -E "Dobot|Camera"
```

### YOLOv9 Issues:
- Ensure model weights are downloaded and path is correct
- Check GPU availability: `nvidia-smi`
- Reduce camera resolution if performance is slow

### Robot Control Issues:
- Verify Dobot is powered on and connected
- Check serial permissions: `sudo chmod 666 /dev/ttyACM*`
- Test manual control first before autonomous mode

### Gazebo Visualization:
- If robot doesn't appear, check URDF mesh paths
- Ensure `magician_description` package is built
- Check Gazebo logs for spawn errors

## Safety Notes

- **Workspace Limits**: Robot movement is constrained to safe bounds
- **Manual Override**: Always test manual control before autonomous operation
- **Emergency Stop**: Keep emergency stop button accessible
- **Camera Placement**: Position camera to have clear view of workspace

## Development

### Adding New Object Classes:
1. Update `target_class` in `yolov9_dobot_controller.py`
2. Ensure YOLOv9 model can detect the target class

### Custom Grippers:
- Modify `pick_object()` method for different end-effectors
- Add gripper-specific commands (open/close)

### Multiple Cameras:
- Modify `camera_index` for different camera devices
- Add camera calibration for accurate coordinate transformation

## How it works

1. **Vision Pipeline**: USB camera captures frames → YOLOv9 detects objects → Pixel coordinates converted to world coordinates
2. **Robot Control**: pydobot library sends commands to physical robot via USB serial
3. **ROS Integration**: Joint states from GUI → Trajectory commands → Gazebo simulation
4. **Dual Control**: Manual control via sliders + autonomous vision-guided picking

The system provides both manual control for testing and autonomous operation for real applications.