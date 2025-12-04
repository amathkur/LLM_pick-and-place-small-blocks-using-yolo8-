# Interactive Robot Training with Demonstrations

This system allows you to train a robot pick-and-place policy by manually demonstrating successful behaviors in RViz.

## How It Works

Instead of pure reinforcement learning exploration, you manually show the robot:
1. Where to pick up objects (by dragging the interactive marker)
2. Where to place objects
3. The system learns from these demonstrations

## Launch Commands

### Full System (Recommended)
```bash
cd /home/abdulhamid/dobot_rviz_ws
ros2 launch launch/full_system.launch.py
```
This launches everything: camera, YOLO detection, joint control GUI, demonstration recorder, and RViz.

### Individual Components
```bash
# Just demonstration training (no camera/YOLO)
ros2 launch launch/demonstration_training.launch.py

# Test YOLO detection only
python3 test_camera_yolo.py
```

## YOLO Object Detection Integration

The system includes YOLOv8 object detection that automatically:
- **Detects objects** of different shapes (cube, cylinder, sphere, etc.)
- **Recognizes colors** (red, green, blue, yellow, orange, purple)
- **Estimates sizes** (small, medium, large) based on distance and pixel size
- **Tracks locations** in 3D space relative to the camera

### Testing YOLO Detection
```bash
python3 test_camera_yolo.py
```
Place colored objects in front of the camera to test detection.

## Training Process with Object Properties

### Step 1: Start Recording
```bash
python3 send_command.py start
```

### Step 2: Position Robot Over Object
- Use the **Joint State Publisher GUI sliders** in RViz to move the robot joints
- Position the end effector directly over the object you want to pick
- Watch RViz to see the robot move in real-time

### Step 3: Record Pick Position with Object Info
- When robot is positioned over object, run:
```bash
python3 send_command.py pick
```
- Enter object properties when prompted:
  - **Size**: 0=small, 1=medium, 2=large
  - **Color**: 0=red, 1=green, 2=blue, 3=yellow
- In RViz, use the Joint State Publisher GUI to position robot over an object
- When prompted, enter the object properties:
  - **Size**: 0=small, 1=medium, 2=large
  - **Color**: 0=red, 1=green, 2=blue, 3=yellow
```bash
python3 send_command.py pick
```

### Step 3: Record Place Position with Target Info
- Move robot to the placement location
- When prompted, specify the target:
  - **Box**: 0=red box, 1=green box, 2=blue box, 3=yellow box
  - **Level**: 0=bottom, 1=middle, 2=top (for stacking)
```bash
python3 send_command.py place
```

### Step 4: Save Demonstration
```bash
python3 send_command.py save
```

## Object and Box System

### Object Properties
- **Sizes**: Small (0), Medium (1), Large (2)
- **Colors**: Red (0), Green (1), Blue (2), Yellow (3)

### Target Boxes
- **Red Box** (0): For red objects
- **Green Box** (1): For green objects  
- **Blue Box** (2): For blue objects
- **Yellow Box** (3): For yellow objects

### Stacking Levels
- **Bottom** (0): Base layer
- **Middle** (1): Second layer
- **Top** (2): Third layer

## Reward System

The trained model learns to:
1. **Pick correct objects**: Bonus for matching size/color
2. **Place in correct boxes**: Higher reward for color-matched placements
3. **Stack properly**: Additional rewards for correct stacking levels

## Example Demonstrations

1. **Pick small red object → Place in red box (bottom)**
   - Size: 0, Color: 0, Box: 0, Level: 0

2. **Pick medium green object → Place in green box (middle)**
   - Size: 1, Color: 1, Box: 1, Level: 1

3. **Pick large blue object → Place in blue box (top)**
   - Size: 2, Color: 2, Box: 2, Level: 2

## Commands Summary

| Command | Description |
|---------|-------------|
| `python3 send_command.py start` | Begin recording a demonstration |
| `python3 send_command.py pick` | Record current marker position as pick location |
| `python3 send_command.py place` | Record current marker position as place location |
| `python3 send_command.py save` | Save the complete demonstration |
| `python3 send_command.py stop` | Stop recording without saving |
| `python3 send_command.py train` | Train model from all saved demonstrations |

## Training for Different Colors, Sizes, and Locations

The robot can be trained to handle objects with different properties in various locations:

### Object Properties
- **Sizes**: Small (0), Medium (1), Large (2)
- **Colors**: Red (0), Green (1), Blue (2), Yellow (3)
- **Locations**: Any position within the robot's workspace

### Training Strategy

1. **Diverse Object Examples**:
   - Record demonstrations with small red objects in different locations
   - Record demonstrations with medium green objects in different locations
   - Record demonstrations with large blue objects in different locations
   - Record demonstrations with yellow objects of various sizes

2. **Location Variation**:
   - Place objects at different distances from the robot
   - Place objects at different angles relative to the robot
   - Use different pickup approaches for different locations

3. **Placement Variation**:
   - Train placing objects in correct color-matched boxes
   - Train stacking at different levels (bottom, middle, top)
   - Train placing objects in different spatial arrangements

### Example Training Scenarios

1. **Color-Matching Task**:
   - Pick red object → Place in red box
   - Pick green object → Place in green box
   - Pick blue object → Place in blue box
   - Pick yellow object → Place in yellow box

2. **Size-Based Task**:
   - Small objects → Bottom level of any box
   - Medium objects → Middle level of any box
   - Large objects → Top level of any box

3. **Location-Based Task**:
   - Objects on left side → Place in left boxes
   - Objects on right side → Place in right boxes
   - Objects at different distances → Adjust approach strategy

### Training Tips

- **Record 5-10 demonstrations** for each object type/color combination
- **Vary object positions** significantly between demonstrations
- **Include edge cases** (objects at workspace boundaries)
- **Test the trained model** with objects in unseen locations
- **Iterate training** if the model doesn't generalize well

## Testing the Trained Model

After training, you can test with:
```bash
python3 test_trained_model.py
```

## Integration with LLM

Once trained, the demonstration model can be used with the LLM controller:
```bash
python3 llm_rl_controller.py
```

The LLM will interpret natural language commands and use the trained policy for execution.

## Files Created

- `demonstrations.json` - Raw demonstration data
- `training_data.json` - Processed training data
- `dobot_demonstration_model.zip` - Trained policy