# Running MoveIt with Gazebo

Due to clock synchronization complexity with the combined launch file, launch them separately:

## Terminal 1: Launch Gazebo
```bash
cd /workspace
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch so_arm_urdf gz_sim.launch.py
```

Wait for Gazebo to fully start and the robot to appear (~10 seconds).

## Terminal 2: Launch MoveIt + RViz
```bash
cd /workspace
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_USE_SIM_TIME=1 ros2 launch so_arm_moveit_config demo.launch.py
```

The `ROS_USE_SIM_TIME=1` environment variable tells ROS2 nodes to use simulation time from Gazebo's `/clock` topic.

## Usage in RViz

1. In the MotionPlanning panel, select a goal state:
   - Set "Select Goal State" to "arm_home" or "arm_ready"
2. Click "Plan & Execute"

The robot should plan and execute the motion in both RViz and Gazebo!

## Notes

- The combined `gazebo_moveit.launch.py` has clock synchronization issues with the MoveIt utility functions
- Launching separately with `ROS_USE_SIM_TIME=1` is the most reliable approach
- Make sure Gazebo is fully started before launching MoveIt

## Troubleshooting

### Clock Synchronization Error

If you see this error:
```
[move_group]: Didn't receive robot state (joint angles) with recent timestamp within 1.000000 seconds.
Requested time 1763411687.991599, but latest received state has time 3341.742000.
```

This means move_group is using wall clock time (~1.7 billion seconds) while joint states use simulation time (~3341 seconds).

**Solution**: Ensure you're using `ROS_USE_SIM_TIME=1` when launching MoveIt:
```bash
ROS_USE_SIM_TIME=1 ros2 launch so_arm_moveit_config demo.launch.py
```

### Joint State Broadcaster Error

If you see:
```
[spawner_joint_state_broadcaster]: Failed to configure controller
```

This is expected - the joint_state_broadcaster is already running from the Gazebo launch and doesn't need to be launched again.
