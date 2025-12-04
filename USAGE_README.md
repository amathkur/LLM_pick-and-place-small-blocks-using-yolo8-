# Dobot MoveIt + LLM Voice/Text + YOLO Integration

## Overview
This system integrates:
- **MoveIt2** - Motion planning for the 4-DOF Dobot arm
- **RViz** - 3D visualization of the robot
- **YOLO** - Trained object detection (shapes/blocks)
- **LLM (Gemini)** - Natural language understanding
- **Voice/Text Control** - Control robot with commands

## Quick Start

### Option 1: Launch Everything (Recommended)
```bash
cd ~/dobot_rviz_ws
./launch_all.sh
```

This opens 2 windows:
1. **RViz + MoveIt** - Robot visualization with camera
2. **LLM Controller** - Text/voice command interface

### Option 2: Manual Launch (Step by Step)

**Terminal 1 - Start RViz and MoveIt:**
```bash
cd ~/dobot_rviz_ws
source install/setup.bash
ros2 launch launch/moveit_demo.launch.py
```

**Terminal 2 - Start LLM Controller:**
```bash
cd ~/dobot_rviz_ws
source install/setup.bash
python3 dobot_llm_project/ros2_llm_controller.py
```

## Usage

### Text Commands
When the LLM controller starts, choose mode `1` for text:
```
Command: move left
Command: go up
Command: move right
Command: go down
Command: go forward
Command: go back
Command: go home
Command: pick bottle
```

### Voice Commands
Choose mode `2` for voice, then speak:
- "move left"
- "go right"  
- "move up"
- "go down"
- "pick the bottle"
- "go home"

### Natural Language (with Gemini)
You can also use natural language:
```
Command: move the robot to the left side
Command: can you go up a little bit?
Command: pick up the red block
Command: return to starting position
```

## What Happens

1. **You speak/type a command** → LLM Controller receives it
2. **Gemini AI understands** → Converts to robot action
3. **MoveIt plans path** → Calculates joint movements
4. **Robot moves in RViz** → You see it move automatically!
5. **YOLO detects objects** → Shows in camera view

## Features

### ✅ Automatic Movement
- **NO manual dragging needed!**
- Just type or speak - robot moves automatically
- Watch it plan and execute in RViz

### ✅ Trained YOLO Detection
- Uses your trained model: `dobot_llm_project/models/yolo_shapes_best.pt`
- Detects shapes, blocks, bottles
- Shows bounding boxes in RViz camera view
- Publishes detections for LLM to use

### ✅ Camera 2 Support
- Uses camera index 2 (your working camera)
- Real-time feed in RViz
- YOLO detection overlay

### ✅ Smart LLM Control
- Understands natural language
- Tracks detected objects
- Plans safe movements
- Can pick objects based on YOLO detections

## System Architecture

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
                    │   Camera 2   │            
                    │   (USB Cam)  │            
                    └──────────────┘            
```

## Configuration

### Change Camera
Edit `camera_publisher.py` line 18:
```python
self.declare_parameter('camera_index', 2)  # Change to 0, 1, 2, etc.
```

### Change YOLO Model
Edit `yolo_detection_node.py` line 21-25:
```python
model_paths = [
    'path/to/your/model.pt',  # Add your model here
    ...
]
```

### Change LLM API Key
Edit `dobot_llm_project/ros2_llm_controller.py` line 42:
```python
self.api_key = "YOUR_GEMINI_API_KEY"
```

## Troubleshooting

### Robot Not Moving
- Check RViz shows "Solution was found and executed" 
- Verify MoveGroup action server is running
- Try simpler commands first ("move up", "go home")

### YOLO Not Detecting
- Check camera feed appears in RViz
- Verify trained model exists: `ls dobot_llm_project/models/*.pt`
- Adjust confidence threshold (default 0.5)

### Voice Not Working
- Install dependencies: `pip install SpeechRecognition pyaudio`
- Check microphone permissions
- Try text mode first to verify system works

### Gemini Not Working
- Check API key is valid
- Verify internet connection
- System falls back to simple keyword matching

## Commands Reference

| Command | Action | Movement |
|---------|--------|----------|
| `move left` | Move in +Y | Robot shifts left |
| `move right` | Move in -Y | Robot shifts right |
| `move up` | Move in +Z | Robot lifts up |
| `move down` | Move in -Z | Robot lowers down |
| `move forward` | Move in +X | Robot extends forward |
| `move back` | Move in -X | Robot retracts back |
| `go home` | Return to home | Robot resets position |
| `pick bottle` | Pick object | Move to detected object |

## Advanced Usage

### Check Topics
```bash
ros2 topic list | grep -E "(yolo|camera)"
```

### View Detections
```bash
ros2 topic echo /yolo/detections
```

### Check Robot State
```bash
ros2 topic echo /joint_states
```

## Files

- `launch_all.sh` - Main launch script
- `launch/moveit_demo.launch.py` - MoveIt + RViz + Camera + YOLO launcher
- `dobot_llm_project/ros2_llm_controller.py` - LLM voice/text controller
- `camera_publisher.py` - Camera node (index 2)
- `yolo_detection_node.py` - YOLO detection with trained model
- `dobot_llm_project/models/yolo_shapes_best.pt` - Trained YOLO model

## Tips

1. **Start with text mode** - Easier to test and debug
2. **Use simple commands first** - "move up", "go down"
3. **Watch RViz** - See robot plan and move automatically
4. **Check camera view** - Should see YOLO bounding boxes
5. **Say "go home"** - Safe reset position anytime

## Support

If something doesn't work:
1. Check both terminal windows for errors
2. Verify camera 2 is connected
3. Ensure RViz is showing the robot
4. Try restarting with `./launch_all.sh`

Happy robot controlling! 🤖🎙️📹
