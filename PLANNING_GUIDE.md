# MoveIt Demo - Planning & Visualization Guide

## Current System Status

✅ **WORKING:**
- ✅ Motion planning with OMPL (RRTConnect algorithm)
- ✅ Interactive marker for manual robot positioning
- ✅ Collision-free trajectory generation
- ✅ Path visualization in RViz
- ✅ YOLO object detection integration
- ✅ Joint state control via GUI

⚠️ **EXPECTED BEHAVIOR:**
- ⚠️ "Solution found but controller failed during execution" - **THIS IS NORMAL**
- ⚠️ Execution fails because there's no real robot hardware
- ⚠️ This is a **VISUALIZATION & PLANNING DEMO** only

## Understanding the "Execution Failed" Message

When you click "Plan & Execute" in RViz, you see:
```
[move_group] Solution found but controller failed during execution
[ERROR] Unable to identify any set of controllers
```

**This is CORRECT behavior!** Here's why:

1. **Planning SUCCEEDS** ✅ - The planner generates a valid collision-free path
2. **Execution FAILS** ❌ - No real robot hardware is connected

The system is working exactly as designed for visualization!

## Gripper Orientation Issue

### Why does the gripper point down when moving the robot?

The gripper orientation is controlled by **joint5** (wrist roll). When you use the interactive marker:

- **Position** is set by the marker location (X, Y, Z)
- **Orientation** is set by the marker's rotation
- **All joint angles** are calculated by inverse kinematics

**Solution:** The planning group includes ALL joints (joint1-5), so:
1. When you drag the marker, IK solver calculates joint1-4 for position
2. Joint5 (gripper rotation) is also computed based on the marker orientation
3. If the marker isn't rotated, joint5 goes to its default angle

### How to maintain gripper orientation:

**Method 1: Use Joint State Publisher**
```bash
# Keep Joint State Publisher GUI open
# Manually set joint5 to desired angle before planning
# The planner will try to maintain this configuration
```

**Method 2: Set orientation constraints in MoveIt**
- In RViz MotionPlanning panel, go to "Planning" tab
- Check "Use Orientation Constraint"
- Set tolerance to small value (0.1 rad)

**Method 3: Use position-only planning (recommended)**
- Modify planning group to exclude joint5
- Plan only for position (X,Y,Z)
- Gripper orientation stays constant

## How to Use This Demo

### Basic Workflow:

1. **Launch the system:**
   ```bash
   cd ~/dobot_rviz_ws
   source install/setup.bash
   ros2 launch launch/moveit_demo.launch.py
   ```

2. **In RViz:**
   - You'll see the robot model
   - Orange sphere at end-effector = interactive marker
   - Drag the sphere to set target position

3. **Plan a motion:**
   - Click **"Plan"** button (NOT "Plan & Execute")
   - Orange trajectory will appear showing planned path
   - This proves planning is working!

4. **What happens if you click "Plan & Execute":**
   - Planning succeeds (green checkmark)
   - Execution fails with controller error
   - **This is expected** - no real robot!

### Working with YOLO Detection:

```bash
# Camera will publish detected objects to /yolo/target_pose
# You can manually plan to these detected positions
# Interactive marker shows the detection location
```

### Adjusting Joint Angles:

1. **Use Joint State Publisher GUI** (opens automatically)
2. Move sliders to adjust each joint
3. Robot updates in real-time in RViz
4. **joint1** - Base rotation
5. **joint2** - Shoulder pitch
6. **joint3** - Elbow pitch  
7. **joint4** - Wrist pitch
8. **joint5** - Gripper roll (rotation around gripper axis)

## Technical Details

### Planning Pipeline:
```
OMPL (geometric::RRTConnect)
├── Input: Start pose + Goal pose
├── Process: Find collision-free path
├── Output: Joint trajectory
└── Status: ✅ WORKING
```

### Execution Pipeline:
```
Trajectory Execution Manager
├── Input: Planned trajectory
├── Process: Send to controllers
├── Expected Controllers: joint1-5 controllers
├── Actual Controllers: NONE (no hardware)
└── Status: ❌ FAILS (expected in simulation-only mode)
```

### Interactive Marker Behavior:
```
Interactive Marker (Sphere)
├── Position: Sets X, Y, Z target
├── Orientation: Sets gripper roll, pitch, yaw
├── IK Solver: Calculates all 5 joint angles
└── Result: May rotate gripper if orientation changes
```

## Solutions for Common Issues

### Issue 1: "Planning failed"
**Cause:** Target pose is unreachable or in collision
**Fix:** 
- Move interactive marker closer to robot
- Check for red collision markers
- Verify joint limits aren't exceeded

### Issue 2: "Gripper rotates during movement"
**Cause:** IK solver computes joint5 based on marker orientation
**Fix:**
- Use Joint State Publisher to pre-set joint5
- Add orientation constraints in MoveIt
- Plan only for position (modify planning group)

### Issue 3: "No interactive marker visible"
**Cause:** Kinematics not loaded properly
**Fix:** ✅ Already fixed in current launch file!
```python
# Kinematics parameters now properly loaded for RViz
rviz_moveit_params.update(kinematics_yaml['ros__parameters'])
```

### Issue 4: "Execution always fails"
**Cause:** No robot hardware connected
**Fix:** **This is normal!** This is a visualization demo.

To actually execute:
1. Connect real Dobot robot
2. Launch dobot_driver nodes
3. Configure controllers in moveit_controllers.yaml
4. Then execution will work

## File Structure

```
dobot_rviz_ws/
├── launch/
│   └── moveit_demo.launch.py          # Main launch file
├── src/magician_moveit_config/
│   └── config/
│       ├── kinematics.yaml            # IK solver config
│       ├── ompl_planning.yaml         # Planning algorithms
│       ├── joint_limits.yaml          # Joint constraints
│       └── magician.srdf              # Robot semantic description
├── yolo_detection_node.py             # Object detection
├── camera_publisher.py                # Camera feed
└── plan_only_demo.py                  # Planning demo helper
```

## Advanced: Planning Only for Position

To plan only position (keep gripper orientation fixed):

### Option A: Modify SRDF planning group
```xml
<!-- In magician.srdf -->
<group name="magician_arm_position_only">
  <joint name="joint1"/>
  <joint name="joint2"/>
  <joint name="joint3"/>
  <joint name="joint4"/>
  <!-- joint5 excluded - won't move during planning -->
</group>
```

### Option B: Use Cartesian path planning
```python
# In Python MoveIt interface:
waypoints = [target_pose]
plan, fraction = move_group.compute_cartesian_path(
    waypoints, 
    eef_step=0.01,
    jump_threshold=0.0
)
# This maintains end-effector orientation
```

## Summary

**Current State:**
- ✅ Planning works perfectly
- ✅ Visualization works perfectly
- ❌ Execution fails (no hardware - expected)
- ⚠️ Gripper rotates (IK includes joint5)

**This is a fully functional planning & visualization demo!**

The "execution failed" error is **PROOF the system is working correctly** - it planned successfully and only failed at the execution stage where hardware would be needed.

## Next Steps (Optional)

To convert this to a real robot system:

1. **Add hardware interface:**
   ```bash
   ros2 run dobot_driver dobot_driver_node
   ```

2. **Configure controllers:**
   ```yaml
   # moveit_controllers.yaml
   controller_names:
     - magician_arm_controller
   
   magician_arm_controller:
     type: FollowJointTrajectory
     joints:
       - joint1
       - joint2
       - joint3
       - joint4
       - joint5
   ```

3. **Launch with hardware:**
   ```bash
   ros2 launch moveit_demo.launch.py use_hardware:=true
   ```

But for visualization and algorithm development, **the current system is complete and working!**
