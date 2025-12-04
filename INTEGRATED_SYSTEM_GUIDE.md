# Integrated Pick-Place System - Complete Guide

## 🎯 System Overview

This system combines:
- **YOLO Object Detection** (99.5% mAP)
- **Depth Sensing** for 3D localization
- **Voice/Text Commands** with optional LLM processing
- **Visual Feedback** in RViz (markers showing detected objects)
- **Camera-Suction Offset Compensation** (50mm automatic)
- **Automatic Home Return** after task completion

## 📐 Current Configuration

```
Suction cup:  55mm from wrist_pitch
Camera:       105mm from wrist_pitch (50mm after suction)
Offset:       50mm (camera ahead of suction)
Base height:  110mm above world frame
```

## 🚀 Quick Start

### 1. Launch the Complete System

```bash
cd ~/dobot_rviz_ws
ros2 launch launch/moveit_demo.launch.py
```

This starts:
- ✅ RViz visualization
- ✅ MoveIt motion planning
- ✅ Camera publisher
- ✅ YOLO detection
- ✅ Transform publishers (with offset compensation)

### 2. Run the Integrated Pick-Place System

In a new terminal:

```bash
cd ~/dobot_rviz_ws
python3 integrated_pick_place_system.py
```

Choose mode:
- **Mode 1 (Text)**: Type commands
- **Mode 2 (Voice)**: Speak commands (requires microphone)

### 3. Test the System

#### Basic Commands:

```bash
# Movement
move left
move right
move up
move down
move forward
move back

# Go home
go home

# Pick and place
pick cube
pick red sphere and place in box a
pick cylinder and place in box b
pick pyramid and place in bin
```

## 🎨 Visual Feedback in RViz

When YOLO detects objects, you'll see in RViz:

1. **Colored Spheres** = Objects detected by camera
   - 🔴 Red = Cube
   - 🟢 Green = Sphere  
   - 🔵 Blue = Cylinder
   - 🟡 Yellow = Pyramid

2. **Purple Arrows** = Target positions for suction cup (offset-compensated)
   - Shows where robot will actually move to pick the object

3. **Green Arrow** = Current robot target pose

## 📊 How Offset Compensation Works

```
Camera Detection → Object at X=0.200m
                ↓
Offset Applied → Suction moves to X=0.150m (0.200 - 0.050)
                ↓
Result → Object centered under suction cup ✓
```

The system automatically:
1. Camera detects object at position (X_cam, Y, Z)
2. Calculates depth from camera to get 3D position
3. Applies 50mm offset: X_suction = X_cam - 0.050m
4. Robot moves suction to compensated position
5. Object is perfectly centered under suction

## 🧠 LLM Integration (Optional)

To enable LLM command processing with Ollama:

### Install Ollama

```bash
curl -fsSL https://ollama.com/install.sh | sh
ollama pull llama2
```

### Enable LLM in System

```bash
python3 integrated_pick_place_system.py --ros-args \
  -p llm_enabled:=true \
  -p llm_api_url:=http://localhost:11434/api/generate
```

With LLM enabled, you can use natural language:
```
"Hey robot, can you grab that red cube and put it in box A?"
"Please move the sphere to box B"
"Put all cubes in the bin"
```

## 🎤 Voice Commands Setup

Install dependencies:

```bash
pip install SpeechRecognition pyaudio
sudo apt-get install portaudio19-dev python3-pyaudio
```

Test microphone:

```bash
python3 -c "import speech_recognition as sr; r = sr.Recognizer(); \
  with sr.Microphone() as source: print('Say something!'); audio = r.listen(source); \
  print('You said:', r.recognize_google(audio))"
```

## 📷 Depth Camera Testing

### Check Depth Data

```bash
# Check if depth topic exists
ros2 topic list | grep depth

# View depth image info
ros2 topic info /camera/depth/image_raw

# Echo depth values
ros2 topic echo /camera/depth/image_raw --once
```

### Verify 3D Position Calculation

Objects should show correct depth in RViz markers:
- Place object 20cm from camera → marker at ~0.20m
- Place object 30cm from camera → marker at ~0.30m

### If Depth Not Available

System will use default depth (30cm) and show warning:
```
[WARN] Invalid depth at (320,240), using default: 0.3m
```

## 🧪 Testing Workflow

### Test 1: Visual Verification in RViz

1. Launch system
2. Place colored object in front of camera
3. Check RViz:
   - ✅ Colored sphere appears at object location
   - ✅ Purple arrow appears 50mm behind sphere
   - ✅ Markers update in real-time

### Test 2: Offset Compensation

1. Place ruler/tape measure in workspace
2. Run: `pick cube`
3. Observe:
   - Camera detects cube at position X
   - Robot moves to X - 50mm
   - Object ends up under suction ✓

### Test 3: Pick and Place

```bash
# Place cube in view
Command: pick cube

# Expected sequence:
📸 Camera sees at: (0.200, 0.000, 0.300)
🔧 Suction moves to: (0.150, 0.000, 0.300)
   ⚙️ Compensated 50mm camera offset
🚀 Moving to: (0.150, 0.000, 0.300) [PICK]
🤏 Closing gripper...
🚀 Moving to: (0.150, 0.000, 0.400) [LIFT]
🚀 Moving to: (0.300, 0.150, 0.150) [PLACE]
✋ Opening gripper...
✅ Task complete! Picked cube → box_a
🏠 Going to home position...
```

### Test 4: Automatic Home Return

After any pick-place task, robot automatically returns to home:
- Home position: (0.2, 0.0, 0.25)
- Status published: `/robot/status` = "at_home"

## 🔧 Troubleshooting

### Camera Not Detected

```bash
# Check available cameras
v4l2-ctl --list-devices

# Update camera index in launch file if needed
# Edit: launch/moveit_demo.launch.py
# Change camera_index parameter
```

### YOLO Not Detecting

```bash
# Check YOLO topic
ros2 topic echo /yolo/detections

# Verify model loaded
# Should see: "YOLO model loaded successfully!"
```

### Offset Incorrect

```bash
# Verify transforms
ros2 run tf2_ros tf2_echo wrist_pitch suction_cup
# Should show: X=0.055

ros2 run tf2_ros tf2_echo suction_cup camera_link  
# Should show: X=0.050

# Total offset = 55mm + 50mm = 105mm from wrist_pitch
```

### Markers Not Showing in RViz

1. Add "MarkerArray" display in RViz
2. Set topic: `/detected_objects_markers`
3. Namespace: `detected_objects` and `suction_targets`

### Voice Recognition Fails

```bash
# Test microphone
arecord -l

# Check permissions
sudo usermod -a -G audio $USER

# Restart and test again
```

## 📝 Configuration Files

### Main Launch File
`~/dobot_rviz_ws/launch/moveit_demo.launch.py`
- Starts all system components
- Configures camera offset (50mm)

### Integrated System
`~/dobot_rviz_ws/integrated_pick_place_system.py`
- Main control node
- Handles commands, vision, motion

### RL Trainer (for learning)
`~/dobot_rviz_ws/dobot_llm_project/rl_pick_place_trainer.py`
- Q-learning training system
- Automatic offset compensation

## 🎯 Real Robot Deployment

When ready to use with real Dobot:

1. **Connect Robot**
   ```bash
   # Serial interface
   python3 dobot_serial_interface.py
   
   # Or CAN interface
   python3 dobot_can_interface.py
   ```

2. **Verify Connection**
   ```bash
   ros2 topic list | grep /dobot
   ```

3. **Run Integrated System**
   ```bash
   python3 integrated_pick_place_system.py
   ```

4. **Test Carefully**
   - Start with small movements
   - Verify offset compensation
   - Test pick-place with light objects first

## 📊 System Architecture

```
                    ┌─────────────────┐
                    │  RViz Display   │
                    │   (markers)     │
                    └────────▲────────┘
                             │
┌──────────┐        ┌────────┴────────┐        ┌──────────┐
│ Camera   │───────>│  Integrated     │───────>│ MoveIt   │
│ (RGB+D)  │        │  Pick-Place     │        │ Planning │
└──────────┘        │     System      │        └──────────┘
                    │                 │
┌──────────┐        │  • Detection    │        ┌──────────┐
│ YOLO     │───────>│  • Depth→3D     │───────>│  Robot   │
│ Detector │        │  • Offset fix   │        │ Hardware │
└──────────┘        │  • Commands     │        └──────────┘
                    └─────────────────┘
                             ▲
                             │
                    ┌────────┴────────┐
                    │ Voice/Text/LLM  │
                    │    Input        │
                    └─────────────────┘
```

## ✅ Success Criteria

System is working correctly when:
- ✅ Markers appear in RViz for detected objects
- ✅ Purple arrows show 50mm behind object markers
- ✅ Robot moves to compensated position
- ✅ Object ends up under suction cup
- ✅ Pick-place completes successfully
- ✅ Robot returns home after task
- ✅ Voice/text commands work reliably

## 🚀 Next Steps

1. **Calibrate precisely** - Measure and verify 50mm offset
2. **Train with RL** - Use rl_pick_place_trainer.py for learning
3. **Add more objects** - Extend YOLO training data
4. **Improve LLM** - Fine-tune prompts for better parsing
5. **Real robot testing** - Deploy to actual Dobot hardware

---

**System Status**: ✅ Ready for testing
**Offset**: ✅ Configured (50mm)
**Visual Feedback**: ✅ Enabled (RViz markers)
**Voice/Text**: ✅ Supported
**Automatic Home**: ✅ Implemented
