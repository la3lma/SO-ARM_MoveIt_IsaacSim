# Running MoveIt with Gazebo (ROS2 Jazzy + Gazebo Harmonic)

Launch Gazebo and MoveIt separately for best results:

## Terminal 1: Launch Gazebo
```bash
cd /workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch so_arm_urdf gz_sim.launch.py
```

Wait for Gazebo to fully start and the robot to appear (~10 seconds).

## Terminal 2: Launch MoveIt + RViz
```bash
cd /workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ROS_USE_SIM_TIME=1 ros2 launch so_arm_moveit_config demo.launch.py
```

The `ROS_USE_SIM_TIME=1` environment variable tells ROS2 nodes to use simulation time from Gazebo's `/clock` topic. ROS2 Jazzy has much better clock synchronization handling than Humble.

## Usage in RViz

1. In the MotionPlanning panel, select a goal state:
   - Set "Select Goal State" to "arm_home" or "arm_ready"
2. Click "Plan & Execute"

The robot should plan and execute the motion in both RViz and Gazebo!

## Notes

- ROS2 Jazzy + Gazebo Harmonic have much better clock synchronization than Humble + Garden
- Launching separately with `ROS_USE_SIM_TIME=1` is recommended
- Make sure Gazebo is fully started before launching MoveIt

## Troubleshooting

### Joint State Broadcaster Error

If you see:
```
[spawner_joint_state_broadcaster]: Failed to configure controller
```

This is expected - the joint_state_broadcaster is already running from the Gazebo launch and doesn't need to be launched again.
