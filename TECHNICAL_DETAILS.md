# Technical Details

Deep dive into the SO-ARM MoveIt + Gazebo integration architecture and implementation decisions.

## Architecture Overview

### Component Stack

```
┌─────────────────────────────────────────┐
│            User Interface               │
│         (RViz + MoveIt Plugin)          │
└─────────────────┬───────────────────────┘
                  │
┌─────────────────▼───────────────────────┐
│           MoveIt2 Framework             │
│  ┌──────────────────────────────────┐  │
│  │      move_group Node             │  │
│  │  - Motion Planning (OMPL)        │  │
│  │  - Trajectory Execution          │  │
│  │  - Planning Scene Monitor        │  │
│  └──────────────┬───────────────────┘  │
└─────────────────┼───────────────────────┘
                  │
┌─────────────────▼───────────────────────┐
│        Controller Manager               │
│     (moveit_simple_controller_manager)  │
└─────────────────┬───────────────────────┘
                  │ /follow_joint_trajectory action
┌─────────────────▼───────────────────────┐
│         ros2_control Framework          │
│  ┌──────────────────────────────────┐  │
│  │   Joint Trajectory Controller    │  │
│  │      (arm_controller)            │  │
│  └──────────────┬───────────────────┘  │
└─────────────────┼───────────────────────┘
                  │
┌─────────────────▼───────────────────────┐
│        Gazebo Harmonic + Plugin         │
│  ┌──────────────────────────────────┐  │
│  │    gz_ros2_control Plugin        │  │
│  │  - Reads URDF ros2_control tags  │  │
│  │  - Provides hardware interface   │  │
│  │  - Simulates joint dynamics      │  │
│  └──────────────────────────────────┘  │
└─────────────────────────────────────────┘
```

## Clock Synchronization

### The Challenge

ROS2 supports two time sources:
- **Wall time**: System clock (real time)
- **Simulation time**: Published on `/clock` topic

For simulation, ALL nodes must use the same time source to ensure:
1. TF transforms have consistent timestamps
2. Trajectory timing is synchronized
3. Action goals don't timeout prematurely

### Our Solution

**Configuration Files**: `ros2_controllers.yaml`
```yaml
controller_manager:
  ros__parameters:
    use_sim_time: true  # Controller manager uses sim time

joint_state_broadcaster:
  ros__parameters:
    use_sim_time: true  # Broadcaster uses sim time

arm_controller:
  ros__parameters:
    use_sim_time: true  # Controller uses sim time
```

**Automatic Runtime Fix**: Integrated in `gazebo_moveit.launch.py`

Why needed: MoveIt's `generate_move_group_launch()` function doesn't properly propagate `use_sim_time` parameter in ROS2 Jazzy. This is a known limitation.

Solution: Launch file includes a `TimerAction` at 18 seconds that automatically sets parameters:
```python
fix_use_sim_time = TimerAction(
    period=18.0,
    actions=[ExecuteProcess(
        cmd=['bash', '-c',
             'ros2 param set /move_group use_sim_time true && '
             'ros2 param set /controller_manager use_sim_time true && '
             # ... (all nodes)
        ]
    )]
)
```

This eliminates the need for manual intervention after launch.

### Clock Flow

```
Gazebo Harmonic
    │
    ├─► /clock topic (publishes sim time)
    │
    ├─► gz_ros_control reads sim time
    │
    ├─► ros2_control uses sim time
    │       │
    │       ├─► arm_controller timestamps
    │       └─► joint_state_broadcaster timestamps
    │
    └─► move_group reads sim time
            │
            └─► Plans and executes with sim time
```

## Joint Configuration

### The Unification Problem

**Original Setup**:
- **Gazebo controller**: Expected 6 joints (including Jaw)
- **MoveIt arm group**: Had only 5 joints (excluded Jaw)
- **Result**: Trajectory rejection - "Joints don't match"

**Solution**: Unified configuration

1. **SRDF arm group** includes all 6 joints:
```xml
<group name="arm">
    <joint name="Rotation"/>
    <joint name="Pitch"/>
    <joint name="Elbow"/>
    <joint name="Wrist_Pitch"/>
    <joint name="Wrist_Roll"/>
    <joint name="Jaw"/>  <!-- Added -->
</group>
```

2. **Controller config** matches:
```yaml
arm_controller:
  ros__parameters:
    joints:
      - Rotation
      - Pitch
      - Elbow
      - Wrist_Pitch
      - Wrist_Roll
      - Jaw  # All 6 joints
```

3. **MoveIt controllers** expects same:
```yaml
moveit_simple_controller_manager:
  arm_controller:
    joints:
      - Rotation
      - Pitch
      - Elbow
      - Wrist_Pitch
      - Wrist_Roll
      - Jaw  # Matches controller
```

### Gripper Handling

The Jaw joint is now part of the arm group, allowing:
- Planning with gripper state
- Coordinated arm+gripper motions
- Simplified controller management

Alternative approach (not used): Separate gripper controller with synchronization.

## Reference Frames

### URDF Structure

```
world (root link)
  │
  └─[fixed joint: world_to_base]
      │
      └─ Base
          │
          └─[revolute: Rotation]
              │
              └─ Rotation_Pitch
                  │
                  └─[revolute: Pitch]
                      │
                      ... (rest of kinematic chain)
```

### MoveIt Frame Configuration

**SRDF Virtual Joint**:
```xml
<virtual_joint name="virtual_joint" type="fixed"
               parent_frame="world" child_link="world"/>
```

**Meaning**:
- **parent_frame="world"**: Planning frame reference
- **child_link="world"**: Robot model root link
- **type="fixed"**: No transform between them

This tells MoveIt:
1. Use "world" as the planning frame
2. Robot model starts at "world" link (matching URDF)
3. Planning frame and model root are at same location

### Why This Configuration?

**Alternative** (doesn't work with our URDF):
```xml
<!-- This would conflict with URDF's world_to_base joint -->
<virtual_joint parent_frame="world" child_link="Base"/>
```

Problem: URDF already has a real `world_to_base` joint. Having both would create:
- Duplicate transforms
- Frame ambiguity
- Transform lookup errors

## Controller Architecture

### Joint Trajectory Controller

**Type**: `joint_trajectory_controller/JointTrajectoryController`

**Functionality**:
1. Subscribes to `/arm_controller/joint_trajectory` topic
2. Provides `/arm_controller/follow_joint_trajectory` action server
3. Interpolates trajectory points
4. Commands joint positions to hardware interface
5. Monitors trajectory execution
6. Reports success/failure

**PID Control Loop** (runs at 100 Hz):
```
Target Position → PID Controller → Commanded Effort → Gazebo Simulation
                      ↑
                      │
                Current Position
```

### Hardware Interface (gz_ros2_control)

**Plugin**: `gz_ros2_control/GazeboSimSystem`

**Responsibilities**:
1. Parse URDF `<ros2_control>` tags
2. Create hardware interface for each joint
3. Read joint states from Gazebo
4. Write joint commands to Gazebo
5. Handle simulation stepping

**Interface Types**:
- **Command Interface**: `position` (what we command)
- **State Interfaces**: `position`, `velocity` (what we read)

## Launch System

### Launch Sequence

```
gazebo_moveit.launch.py
    │
    ├─► [t=0s] Gazebo (gz_sim.launch.py)
    │     │
    │     ├─► Load world
    │     ├─► Spawn robot from URDF
    │     └─► Load gz_ros2_control plugin
    │           │
    │           └─► Initialize controllers
    │
    ├─► [t=12s] MoveIt (move_group.launch.py)
    │     │
    │     ├─► Load robot_description
    │     ├─► Load SRDF
    │     ├─► Start move_group node
    │     └─► Connect to controllers
    │
    ├─► [t=15s] RViz (moveit_rviz.launch.py)
    │     │
    │     └─► Load MoveIt plugin
    │
    └─► [t=18s] Automatic use_sim_time Fix
          │
          └─► Set use_sim_time=true for all nodes
```

### Why Delays?

**12 second delay for move_group**:
- Gazebo needs time to:
  - Start physics engine
  - Load robot model
  - Initialize ros2_control plugin
  - Activate controllers

**15 second delay for RViz**:
- move_group needs time to:
  - Load planning pipelines
  - Build kinematic model
  - Start action servers
  - Connect to controllers

**18 second delay for automatic fix**:
- Ensures move_group is fully initialized before setting parameters
- Buffer time after move_group start (12s + 6s)
- Prevents "Node not found" errors

Without delays: Race conditions cause initialization failures.

## Trajectory Execution Flow

### Complete Motion Execution Path

1. **User** sets goal in RViz
2. **MoveGroupInterface** creates motion plan request
3. **move_group** node:
   - Validates start state
   - Calls OMPL planner
   - Generates trajectory
   - Time-parameterizes trajectory
4. **TrajectoryExecutionManager** validates trajectory
5. **ControllerManager** sends trajectory to action server
6. **arm_controller** receives trajectory:
   - Validates joint names match
   - Interpolates waypoints
   - Executes PID control loop
7. **gz_ros2_control** plugin:
   - Receives position commands
   - Applies forces in Gazebo
8. **Gazebo** simulates physics:
   - Integrates dynamics
   - Updates joint positions
9. **gz_ros2_control** reads new states
10. **joint_state_broadcaster** publishes to `/joint_states`
11. **RViz** visualizes updated robot state

### Action Server Communication

```
MoveIt                Controller
  │                      │
  │─────[Goal]──────────►│
  │                      │ Accept
  │◄────[Accepted]───────│
  │                      │ Executing...
  │◄────[Feedback]───────│ (periodic)
  │◄────[Feedback]───────│
  │◄────[Feedback]───────│
  │                      │ Done
  │◄────[Result]─────────│
  │     (SUCCESS)        │
```

## Known Issues & Workarounds

### Issue 1: move_group use_sim_time

**Problem**: `generate_move_group_launch()` doesn't respect `use_sim_time` parameter.

**Root Cause**: MoveIt launch utilities in ROS2 Jazzy have limited parameter propagation.

**Solution**: Automatic runtime parameter setting integrated in `gazebo_moveit.launch.py` via `TimerAction` at 18 seconds.

**Status**: ✅ Resolved - No manual intervention required.

### Issue 2: Transform Warning in RViz

**Problem**: RViz shows `Given transform is to frame 'Base', but frame 'world' was expected`

**Root Cause**: RViz's planning scene display interprets frame hierarchy differently than MoveIt core.

**Impact**: Cosmetic only - doesn't affect planning or execution.

**Status**: Low priority - execution works correctly.

### Issue 3: Startup Timing Sensitivity

**Problem**: Startup order and timing critical for success.

**Root Cause**: Complex initialization dependencies between components.

**Mitigation**: Fixed delays in launch files + runtime fix script.

**Alternative**: Event-driven launch with state monitoring (complex).

## Performance Characteristics

### Planning Time

- **Simple motions**: 50-200ms
- **Complex motions**: 200-1000ms
- **Timeout**: 5 seconds (configurable)

### Execution Time

- Depends on:
  - Trajectory length
  - Joint velocity limits
  - Acceleration limits
  - Controller update rate (100 Hz)

### Resource Usage

**Typical Load**:
- **CPU**: 30-50% (4 cores)
- **RAM**: 2-3 GB
- **GPU**: 20-40% (for Gazebo rendering)

## Extending the System

### Adding New Planning Group

1. Edit SRDF - add new group
2. Configure kinematics solver
3. Update joint limits
4. Test in RViz

### Adding Sensors

1. Add sensor to URDF (e.g., camera, lidar)
2. Configure Gazebo sensor plugin
3. Add ros_gz_bridge for sensor topics
4. Update MoveIt sensor config

### Custom Controllers

1. Create controller plugin
2. Register with controller_manager
3. Configure in ros2_controllers.yaml
4. Test with ros2_control CLI

### Python/C++ Scripts

Use MoveIt's Python or C++ API:

```python
from moveit_commander import MoveGroupCommander
move_group = MoveGroupCommander("arm")
move_group.set_named_target("arm_home")
move_group.go(wait=True)
```

## Development Best Practices

### Debugging

1. **Use logging**: Check `/workspace/logs/`
2. **Monitor topics**: `ros2 topic echo`
3. **Check parameters**: `ros2 param list/get`
4. **Test controllers**: `ros2 control` CLI
5. **Profile**: Use `ros2 topic hz` for rates

### Testing Changes

1. Build with `colcon build --packages-select <pkg>`
2. Source workspace
3. Test individual components first
4. Full integration test last

### Version Control

- Commit working configurations
- Tag stable releases
- Document breaking changes
- Keep launch files in sync

## References

### Documentation

- [MoveIt2 Tutorials](https://moveit.picknik.ai/main/doc/tutorials/tutorials.html)
- [ros2_control Documentation](https://control.ros.org/)
- [gz_ros2_control](https://github.com/ros-controls/gz_ros2_control)
- [Gazebo Harmonic](https://gazebosim.org/docs/harmonic)

### Source Code

- [MoveIt2 GitHub](https://github.com/ros-planning/moveit2)
- [ros2_control GitHub](https://github.com/ros-controls/ros2_control)
- [ros2_controllers GitHub](https://github.com/ros-controls/ros2_controllers)

### Key Concepts

- [ROS2 Time Synchronization](https://design.ros2.org/articles/clock_and_time.html)
- [TF2 in ROS2](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Tf2-Main.html)
- [Action Servers](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html)
