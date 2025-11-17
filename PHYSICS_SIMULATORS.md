# Physics Simulators Setup Guide

This guide covers setting up physics simulation for your SO-ARM robot using either Gazebo or Isaac Sim.

## Option 1: Gazebo Garden (Recommended - Working!)

### ✅ Current Status: Working with Gazebo Garden

The SO-ARM robot now successfully spawns and simulates in **Gazebo Garden (gz-sim)** with proper physics!

**What's Working:**
- Robot visualizes correctly with STL meshes
- Physics simulation (gravity, collisions, joint dynamics)
- ROS2 integration via ros_gz bridge
- Joint state publisher GUI for manual control

### Quick Launch

**In VNC terminal:**

```bash
cd /workspace
source install/setup.bash

# Launch Gazebo with robot (includes physics simulation)
ros2 launch so_arm_urdf gz_sim.launch.py
```

**What you'll see:**
1. **Gazebo window** - 3D physics simulator with SO-ARM robot
2. **Joint State Publisher GUI** - Sliders for 6 joints
3. Robot responding to gravity and physics

### How It Works

The launch file (`gz_sim.launch.py`) does several things:

1. **Launches Gazebo with ROS2 integration** - Uses `ros_gz_sim` launch file for proper ROS-Gazebo communication
2. **Converts mesh paths** - Automatically converts `package://` URIs to `file://` paths that Gazebo can resolve
3. **Spawns the robot** - Waits 8 seconds for Gazebo to initialize, then spawns the robot at (0, 0, 0.2)
4. **Starts bridges** - Creates ROS2 ↔ Gazebo bridges for clock and other topics
5. **Launches GUI** - Opens Joint State Publisher GUI for manual control

### Known Issues and Fixes

**Issue: Robot tips over when simulation starts**

**Cause:** The base link isn't fixed to the ground, so the robot can move freely and falls over under gravity.

**Solution:** The URDF includes a `world` link with a fixed joint to the `Base` link, anchoring the robot to the ground. This simulates how a real robot arm would be bolted to a table/floor.

```xml
<link name="world"/>
<joint name="world_to_base" type="fixed">
  <parent link="world"/>
  <child link="Base"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>
```

**Alternative:** If the robot still tips, you can also:
- Increase the base mass in the URDF
- Add joint damping to all joints
- Set joint limits to prevent extreme movements

### Connect Gazebo to MoveIt

**Terminal 1** - Launch Gazebo:
```bash
ros2 launch so_arm_urdf gazebo.launch.py
```

**Terminal 2** - Launch MoveIt:
```bash
ros2 launch so_arm_moveit_config move_group.launch.py
```

**Terminal 3** - Launch RViz:
```bash
ros2 launch so_arm_moveit_config moveit_rviz.launch.py
```

Now you can plan motions in RViz and execute them in Gazebo!

### Troubleshooting Gazebo

**Gazebo window is black/frozen:**
```bash
# Check if Gazebo is running
ps aux | grep gazebo

# Kill and restart
pkill -9 gzserver gzclient
ros2 launch so_arm_urdf gazebo.launch.py
```

**Robot falls through ground:**
- Check URDF has proper collision geometries
- Verify inertial properties are set

**Joints don't move:**
- Ensure `gazebo_ros2_control` plugin is in URDF
- Check controller configuration

---

## Option 2: Isaac Sim (Advanced - GPU Required)

Your project already has USD files for Isaac Sim! (`src/so_arm_description/usd_file/`)

### Prerequisites

- **NVIDIA GPU** (RTX series recommended)
- **NVIDIA drivers** installed on host
- **nvidia-docker** runtime

### Installation Options

#### A. Isaac Sim on Host Machine

1. **Download Isaac Sim**:
   ```bash
   # Visit: https://developer.nvidia.com/isaac-sim
   # Download Isaac Sim 2023.1.1 or later
   ```

2. **Install**:
   ```bash
   # Extract and run installer
   ./isaac-sim.sh --install
   ```

3. **Run Isaac Sim**:
   ```bash
   ./isaac-sim.sh
   ```

#### B. Isaac Sim in Docker (Requires GPU Passthrough)

Update `docker-compose.yml` to use the GPU-enabled service:

```bash
# On host, ensure nvidia-docker is installed
docker compose --profile gpu up ros2-dev-gpu
```

### Using Isaac Sim with ROS2

#### Step 1: Load Robot in Isaac Sim

1. Open Isaac Sim
2. **File → Open** → Navigate to `workspace/src/so_arm_description/usd_file/`
3. Load `so_arm_urdf_usd.usd` or `ActionGraph.usd`

#### Step 2: Enable ROS2 Bridge

In Isaac Sim:
1. **Window → Extensions**
2. Search for "ROS2 Bridge"
3. Enable it
4. Configure:
   - **ROS Domain ID**: 0 (match your container)
   - **Clock Topic**: `/clock`
   - **TF Prefix**: (leave empty)

#### Step 3: Setup Action Graph

The `ActionGraph.usd` file contains:
- Joint state publishers
- TF broadcasters
- Control subscribers

Load it to enable ROS2 communication.

#### Step 4: Connect to MoveIt

**Terminal 1** (in container) - Launch MoveIt:
```bash
cd /workspace
source install/setup.bash
ros2 launch so_arm_moveit_config move_group.launch.py
```

**Terminal 2** - Launch RViz:
```bash
ros2 launch so_arm_moveit_config moveit_rviz.launch.py
```

**Terminal 3** - Verify communication:
```bash
# Check topics
ros2 topic list | grep joint

# Should see Isaac Sim publishing:
# /joint_states
# /tf
# /tf_static
```

Now plan in MoveIt and execute in Isaac Sim with full physics!

### Isaac Sim Advantages

- **Photorealistic rendering** - Camera simulation, lighting
- **Advanced physics** - PhysX engine, soft bodies, fluids
- **Synthetic data** - Perfect for training ML/AI models
- **GPU acceleration** - Much faster than Gazebo
- **ROS2 native** - Better integration than Gazebo Classic

### Isaac Sim Networking

If running Isaac Sim on host and ROS2 in Docker:

1. **Set ROS Domain ID** (both must match):
   ```bash
   # In container
   export ROS_DOMAIN_ID=0

   # On host (before starting Isaac Sim)
   export ROS_DOMAIN_ID=0
   ```

2. **Test communication**:
   ```bash
   # In container
   ros2 topic list

   # On host
   ros2 topic list

   # Should see the same topics
   ```

---

## Comparison: Gazebo vs Isaac Sim

| Feature | Gazebo | Isaac Sim |
|---------|--------|-----------|
| **Setup Time** | 5 minutes | 1-2 hours |
| **GPU Required** | No | Yes (NVIDIA) |
| **Physics Engine** | ODE/Bullet/DART | PhysX (NVIDIA) |
| **Rendering** | Basic OpenGL | Photorealistic RTX |
| **ROS2 Integration** | Native (ros-gz) | ROS2 Bridge |
| **Performance** | CPU-based | GPU-accelerated |
| **Cameras/Sensors** | Basic | Advanced (RTX, lidar, etc.) |
| **Learning Curve** | Easy | Moderate |
| **Cost** | Free | Free (for development) |
| **Best For** | Quick testing, prototyping | ML/AI training, high-fidelity sim |

---

## Recommended Path

1. **Start with Gazebo** - Get familiar with physics simulation
2. **Test MoveIt integration** - Verify motion planning works
3. **Move to Isaac Sim** - When you need advanced features

---

## Next Steps After Physics Sim

Once you have physics simulation working:

### 1. Add Sensors
```python
# Add camera to URDF/USD
<sensor name="camera" type="camera">
  <update_rate>30</update_rate>
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
    </image>
  </camera>
</sensor>
```

### 2. Add Controllers
```bash
# ros2_control configuration
ros2 control load_controller joint_trajectory_controller
ros2 control switch_controllers --start joint_trajectory_controller
```

### 3. Create Task Scripts
```python
# Example: Pick and place
from moveit_py import MoveItPy, PlanningComponent

# Plan to pre-grasp
plan = planning_component.plan()

# Execute in sim
moveit_py.execute(plan)
```

### 4. Integrate Perception
- Object detection
- Point cloud processing
- Grasp pose estimation

### 5. Real Robot Transfer
- Test in sim first
- Validate trajectories
- Deploy to hardware

---

## Troubleshooting Both Simulators

### DDS/ROS2 Communication Issues

```bash
# Check DDS discovery
ros2 doctor --report

# List all nodes
ros2 node list

# Check specific topic
ros2 topic info /joint_states -v

# Echo to verify data
ros2 topic echo /joint_states --once
```

### Performance Issues

**Gazebo:**
```xml
<!-- In world file, reduce real-time factor -->
<physics type="ode">
  <real_time_update_rate>100</real_time_update_rate>
  <max_step_size>0.01</max_step_size>
</physics>
```

**Isaac Sim:**
- Lower rendering quality in settings
- Reduce physics timestep
- Disable shadows/reflections

### Synchronization Issues

```bash
# Use sim time
ros2 param set /move_group use_sim_time true
ros2 param set /robot_state_publisher use_sim_time true
```

---

## Resources

**Gazebo:**
- [Gazebo-ROS2 Tutorials](https://gazebosim.org/docs)
- [ros2_control + Gazebo](https://control.ros.org/master/doc/gazebo_ros2_control/doc/index.html)

**Isaac Sim:**
- [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/)
- [ROS2 Bridge Tutorial](https://docs.omniverse.nvidia.com/isaacsim/latest/ros2_tutorials/index.html)
- [USD Asset Creation](https://docs.omniverse.nvidia.com/isaacsim/latest/advanced_tutorials/tutorial_advanced_import_urdf.html)

**Your Project:**
- USD files: `src/so_arm_description/usd_file/`
- URDF: `src/so_arm_description/urdf/so_arm_urdf.urdf`
- MoveIt config: `src/so_arm_moveit_config/`

Good luck with your physics simulation!
