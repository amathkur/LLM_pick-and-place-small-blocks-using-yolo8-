# Camera and Suction Cup Calibration Guide

## Physical Setup (As Per Your Specifications)

```
Robot End Effector Chain:
wrist_pitch (last joint)
    ↓ +35mm along Z axis
suction_cup (picking point)
    ↓ +25mm along Z axis  
camera_link (camera center, looking DOWN at suction cup)

Total: 60mm from wrist_pitch to camera
```

## Transform Configuration

The launch file now includes:

1. **Suction Cup Transform** (`wrist_pitch` → `suction_cup`):
   - Translation: [0.035, 0.0, 0.0] meters (35mm forward)
   - Rotation: None (same orientation as wrist_pitch)

2. **Camera Transform** (`suction_cup` → `camera_link`):
   - Translation: [0.025, 0.0, 0.0] meters (25mm forward from suction)
   - Rotation: -90° pitch (looking DOWN at suction point)
   - Quaternion: [0, -0.7071068, 0, 0.7071068]

## Coordinate System Analysis

### Camera Frame (camera_link)
When camera looks DOWN:
- **+X**: Points to the RIGHT in camera image
- **+Y**: Points DOWN in camera image  
- **+Z**: Points OUT from camera (optical axis - pointing DOWN towards table)

### Detection Coordinates
Objects detected in camera will be:
- **Center of image**: X=0, Y=0 (directly below camera/suction)
- **Right side**: X > 0
- **Left side**: X < 0
- **Bottom of image**: Y > 0
- **Top of image**: Y < 0

### Robot Base Frame Relationship
When robot moves:
- **Joint 1 rotation RIGHT** → Camera sees object move LEFT in frame (X becomes negative)
- **Joint 1 rotation LEFT** → Camera sees object move RIGHT in frame (X becomes positive)

## Expected Behavior

1. **Object in center of camera**: 
   - Detection: X ≈ 0, Y ≈ 0
   - This means object is directly below suction cup (perfect alignment)

2. **Robot moves right (+Joint1)**:
   - Object moves left in camera view (X becomes negative)
   - To re-center: Robot must move LEFT (decrease Joint1)

3. **Robot moves left (-Joint1)**:
   - Object moves right in camera view (X becomes positive)
   - To re-center: Robot must move RIGHT (increase Joint1)

## Depth (Z) Calculation Problem

Current YOLO node uses **fixed distance = 0.5m** which is INCORRECT for:
- Different object heights
- Camera-to-table distance varies with robot arm angles
- No accounting for 60mm camera offset

### Solution: Use Camera Height Above Table

For accurate picking, Z should be:
```
Z_table = camera_height_above_table - 0.060  # Subtract camera offset to get suction height
```

## Calibration Steps

### Step 1: Measure Camera Height
```bash
# With robot in home position (all joints = 0)
# Measure from table to camera lens
# Example: If camera is 30cm above table
# Then suction cup is at: 30cm - 6cm = 24cm above table
```

### Step 2: Place Object at Known Position
```bash
# Place a cube at known position (e.g., 20cm in front of robot base)
# Mark its exact position
```

### Step 3: Check Detection Coordinates
```bash
# Run system and check detection output
ros2 topic echo /yolo/detections

# Compare detected X, Y with actual position
# Adjust if needed
```

### Step 4: Test Robot Motion
```bash
# In RViz, use interactive markers to move robot
# Object should stay centered as robot moves side-to-side
# If object appears to "slide" in camera view, check:
#   - Camera rotation (must be exactly -90° pitch)
#   - TF tree (verify wrist_pitch → suction_cup → camera_link)
```

## Verification Commands

```bash
# Check TF tree
ros2 run tf2_tools view_frames
evince frames.pdf

# Verify camera transform
ros2 run tf2_ros tf2_echo wrist_pitch camera_link

# Check current detection
ros2 topic echo /yolo/target_pose

# Visualize in RViz:
# - Add TF display (see all frames)
# - Add Camera display (see camera view)
# - Add Marker display (see detection markers)
# - Add InteractiveMarkers (move robot with mouse)
```

## Common Issues & Fixes

### Issue 1: "Object appears behind robot"
**Cause**: Camera optical axis not pointing down, or camera frame Z-axis pointing wrong direction
**Fix**: Verify rotation quaternion is [0, -0.7071068, 0, 0.7071068] for -90° pitch

### Issue 2: "Object moves opposite direction when robot moves"
**Cause**: Camera X or Y axis inverted
**Fix**: Check camera frame definition, may need to add 180° yaw rotation

### Issue 3: "Detection not centered when suction is centered"
**Cause**: Camera offset not accounted for, or wrong parent frame
**Fix**: Ensure camera is child of suction_cup frame with correct 25mm offset

### Issue 4: "Depth (Z) always wrong"
**Cause**: Using fixed 0.5m distance instead of actual measurement
**Fix**: Update YOLO node to calculate Z based on robot forward kinematics

## RViz Interactive Testing

1. Open RViz (should already be open)
2. Add **InteractiveMarker** display
   - Topic: `/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic`
3. Click and drag the blue/green/red arrows to move robot
4. Watch camera view - object should:
   - Stay centered when moving vertically (joint 2, 3)
   - Move left in view when robot moves right (joint 1)
   - Stay at same depth when rotating base

## Next Steps

After verifying transforms are correct:
1. Update YOLO node to calculate real Z based on robot pose
2. Add dynamic depth estimation (if needed)
3. Test pick-and-place with real objects
4. Fine-tune gripper offsets if needed
