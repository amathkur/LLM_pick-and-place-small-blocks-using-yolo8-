#!/bin/bash

# YOLOv9 Setup Script for Dobot Robot Control

echo "Setting up YOLOv9 for Dobot robot control..."

# Install Python dependencies
echo "Installing Python dependencies..."
pip3 install -r requirements.txt

# Download YOLOv9 model weights (you'll need to choose appropriate weights)
echo "Please download YOLOv9 model weights from:"
echo "https://github.com/WongKinYiu/yolov9/releases"
echo ""
echo "Common options:"
echo "- yolov9-c.pt (small model, fast)"
echo "- yolov9-e.pt (large model, accurate)"
echo ""
echo "Place the downloaded weights file at: /path/to/yolov9/weights/yolov9.pt"
echo "Then update the model_path in yolov9_dobot_controller.py"

# Install ROS2 dependencies
echo "Installing ROS2 dependencies..."
sudo apt-get update
sudo apt-get install -y ros-humble-cv-bridge ros-humble-image-transport

# Check for USB camera
echo "Checking for USB camera..."
if ls /dev/video* 1> /dev/null 2>&1; then
    echo "USB camera(s) found:"
    ls /dev/video*
else
    echo "No USB cameras found. Please connect a USB camera."
fi

# Check for Dobot serial ports
echo "Checking for Dobot serial ports..."
if ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null; then
    echo "Serial ports found:"
    ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
else
    echo "No serial ports found. Please connect the Dobot robot."
fi

echo ""
echo "Setup complete! Please:"
echo "1. Download YOLOv9 weights and update the path in yolov9_dobot_controller.py"
echo "2. Connect USB camera and Dobot robot"
echo "3. Run the launch file: ros2 launch rviz_gazebo_control.launch.py"