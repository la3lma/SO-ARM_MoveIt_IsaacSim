# SO-ARM Setup Guide

Detailed setup instructions for ROS2 Jazzy + MoveIt + Gazebo Harmonic integration.

## Table of Contents

1. [Initial Setup](#initial-setup)
2. [Docker Environment](#docker-environment)
3. [Building the Workspace](#building-the-workspace)
4. [Running the System](#running-the-system)
5. [Verification](#verification)

## Initial Setup

### System Requirements

- **OS**: Linux (Ubuntu 22.04+ recommended) or macOS with X11
- **Docker**: Version 20.10+
- **Docker Compose**: Version 2.0+
- **RAM**: Minimum 8GB (16GB recommended)
- **Display**: X11 server for GUI

### Clone Repository

```bash
git clone <repository-url>
cd SO-ARM_MoveIt_IsaacSim
```

## Docker Environment

### Build Container

The Docker image includes:
- ROS2 Jazzy (latest)
- MoveIt2
- Gazebo Harmonic
- ros2_control and gz_ros2_control
- All necessary dependencies

```bash
# Build the image (takes 10-15 minutes first time)
make build

# Or using docker-compose directly:
docker-compose build ros2-moveit
```

### Start Container

```bash
# Start container with X11 forwarding
make up

# Or using docker-compose:
docker-compose up -d ros2-moveit
```

### Access Container

```bash
# Enter container shell
docker exec -it so-arm-moveit bash

# Or use make shortcut:
make shell
```

## Building the Workspace

### Initial Build

Inside the container:

```bash
cd /workspace
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Build time: ~2-3 minutes

### Verify Build

```bash
# Check packages
ros2 pkg list | grep so_arm

# Should output:
# so_arm_moveit_config
# so_arm_urdf
```

## Running the System

### Method 1: Using Launch Script (Recommended)

The launch script automatically:
- Sources ROS environment
- Launches Gazebo + MoveIt + RViz
- Applies clock synchronization fix automatically
- Logs all output to timestamped file

```bash
cd /workspace
./launch_and_log.sh
```

**Wait for initialization** (~20 seconds). You should see:
1. Gazebo window with robot loaded
2. RViz window with MoveIt interface
3. Automatic clock sync fix message: "✓ use_sim_time parameters applied successfully"

### Method 2: Manual Launch

```bash
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
ros2 launch so_arm_moveit_config gazebo_moveit.launch.py
```

The launch system automatically applies the `use_sim_time` parameter fix at 18 seconds after launch, eliminating the need for manual intervention

## Verification

### 1. Check Nodes

```bash
ros2 node list
```

**Expected nodes**:
```
/arm_controller
/controller_manager
/gz_ros_control
/move_group
/robot_state_publisher
/ros_gz_bridge
/rviz
```

### 2. Check Controllers

```bash
ros2 control list_controllers
```

**Expected output**:
```
arm_controller[joint_trajectory_controller/JointTrajectoryController] active
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
```

### 3. Check Topics

```bash
# Joint states should be publishing
ros2 topic hz /joint_states
# Should show ~100 Hz

# Clock topic for simulation time
ros2 topic echo /clock --once
# Should show incrementing simulation time
```

### 4. Check Parameters

```bash
# All should return "Boolean value is: True"
ros2 param get /move_group use_sim_time
ros2 param get /controller_manager use_sim_time
ros2 param get /arm_controller use_sim_time
ros2 param get /robot_state_publisher use_sim_time
```

### 5. Test Motion Planning

In RViz:

1. **Select planning group**: Choose "arm" from dropdown
2. **Set goal state**:
   - Drag interactive marker to new position
   - Or use "Select Goal State" → pick a saved state
3. **Plan**: Click "Plan" button
   - Orange trajectory should appear
   - Check terminal for planning success message
4. **Execute**: Click "Execute" button
   - **Robot should move in BOTH RViz and Gazebo**
   - Check logs for "Goal reached, success!"

### 6. Verify Logs

```bash
# Check latest log file
ls -lt /workspace/logs/*.log | head -1

# Search for success message
grep "Goal reached, success" /workspace/logs/gazebo_moveit_*.log
```

## Troubleshooting

### Container Won't Start

**Issue**: Display connection error

**Solution**:
```bash
# On host (macOS)
xhost + localhost

# On host (Linux)
xhost +local:docker
```

### Build Fails

**Issue**: Dependency errors

**Solution**:
```bash
# Update rosdep
sudo rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Clean and rebuild
rm -rf build/ install/ log/
colcon build --symlink-install
```

### Gazebo Window Black/Empty

**Issue**: GPU acceleration problem

**Solution**:
```bash
# Set software rendering
export LIBGL_ALWAYS_SOFTWARE=1

# Restart Gazebo
```

### Controllers Not Active

**Issue**: Controllers fail to load

**Solution**:
```bash
# Check Gazebo ros2_control plugin loaded
ros2 node info /gz_ros_control

# Try loading controllers manually
ros2 control load_controller arm_controller
ros2 control set_controller_state arm_controller start
```

### use_sim_time Still False

**Issue**: Parameter not propagating or automatic fix didn't run

**Solution**:
1. Ensure you waited at least 20 seconds after launch
2. Look for "✓ use_sim_time parameters applied successfully" in logs
3. If automatic fix didn't run, manually set parameters:
   ```bash
   ros2 param set /move_group use_sim_time true
   ros2 param set /controller_manager use_sim_time true
   ros2 param set /arm_controller use_sim_time true
   ros2 param set /joint_state_broadcaster use_sim_time true
   ros2 param set /robot_state_publisher use_sim_time true
   ```

### Motion Executes in RViz but Not Gazebo

**Issue**: Controller not receiving commands

**Solution**:
1. Check action server exists:
   ```bash
   ros2 action list | grep follow_joint_trajectory
   ```

2. Monitor action feedback:
   ```bash
   ros2 topic echo /arm_controller/follow_joint_trajectory/_action/feedback
   ```

3. Check for joint mismatch errors in logs:
   ```bash
   grep "don't match" /workspace/logs/gazebo_moveit_*.log
   ```

## Advanced Configuration

### Modify Joint Limits

Edit: `src/so_arm_moveit_config/config/joint_limits.yaml`

```yaml
joint_limits:
  Rotation:
    max_velocity: 1.0        # rad/s
    max_acceleration: 1.0    # rad/s²
```

Rebuild after changes:
```bash
colcon build --packages-select so_arm_moveit_config
```

### Adjust Controller PID Gains

Edit: `src/so_arm_description/config/ros2_controllers.yaml`

```yaml
arm_controller:
  ros__parameters:
    gains:
      Rotation:
        p: 100.0
        i: 0.01
        d: 10.0
```

Restart Gazebo to apply changes.

### Change Planning Algorithm

Edit: `src/so_arm_moveit_config/config/ompl_planning.yaml`

Available planners:
- RRTConnect (default - fast, non-optimal)
- RRT* (optimal, slower)
- PRM (probabilistic roadmap)

### Add Obstacles

In RViz:
1. Scene Objects panel
2. Import mesh or use primitive shapes
3. Publish planning scene

Or programmatically:
```python
from moveit_msgs.msg import PlanningScene
# Add collision objects to planning scene
```

## Performance Tuning

### Reduce Simulation Overhead

```yaml
# In ros2_controllers.yaml
controller_manager:
  ros__parameters:
    update_rate: 50  # Reduce from 100 Hz if needed
```

### Faster Planning

```yaml
# In ompl_planning.yaml
arm:
  planner_configs:
    - RRTConnect
  projection_evaluator: joints(Rotation,Pitch)
  longest_valid_segment_fraction: 0.01  # Increase for faster planning
```

## Next Steps

1. **Test different planning algorithms**: Edit OMPL configuration
2. **Add collision objects**: Use RViz Scene Objects
3. **Create motion sequences**: Write Python/C++ MoveIt scripts
4. **Tune controller gains**: Adjust PID values for smoother motion
5. **Implement grasping**: Integrate gripper control logic

## Support

For issues:
1. Check `/workspace/logs/` for detailed error messages
2. Review troubleshooting section
3. Verify all verification steps pass
4. Check ROS2/MoveIt/Gazebo documentation

## Additional Resources

- [MoveIt Tutorials](https://moveit.picknik.ai/main/doc/tutorials/tutorials.html)
- [ros2_control Tutorials](https://control.ros.org/master/doc/getting_started/getting_started.html)
- [Gazebo Tutorials](https://gazebosim.org/docs/harmonic/tutorials)
