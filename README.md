# SO-ARM MoveIt + Gazebo Integration

Complete ROS2 Jazzy + MoveIt + Gazebo Harmonic integration for the SO-ARM robotic arm.

## Status

✅ **Working** - Robot motion planning and execution in both RViz and Gazebo Harmonic

## Features

- **ROS2 Jazzy** with latest improvements
- **MoveIt2** motion planning with OMPL
- **Gazebo Harmonic** physics simulation
- **ros2_control** hardware abstraction
- Full 6-DOF arm control (including gripper jaw)
- Synchronized simulation time across all nodes
- Automated logging for debugging

## Quick Start

### Prerequisites

- Docker and Docker Compose installed
- X11 forwarding configured (for GUI)

### Launch

```bash
# Build and start container
make up

# In container, launch Gazebo + MoveIt + RViz
docker exec -it so-arm-moveit bash
cd /workspace
./launch_and_log.sh
```

**Wait ~20 seconds** for all components to initialize (including automatic clock sync fix).

### Plan and Execute Motion

1. In RViz, use the MotionPlanning plugin
2. Drag the interactive marker to set a goal pose
3. Click "Plan & Execute"
4. **Robot will move in both RViz and Gazebo!**

## Documentation

- **[SETUP_GUIDE.md](SETUP_GUIDE.md)** - Detailed setup and verification steps
- **[TECHNICAL_DETAILS.md](TECHNICAL_DETAILS.md)** - Architecture and implementation details

## Logs

All output is automatically logged to `/workspace/logs/gazebo_moveit_TIMESTAMP.log`

## Quick Troubleshooting

### Robot doesn't move

1. Ensure you waited ~20 seconds for all systems to initialize
2. Check `use_sim_time` parameters are `true`:
   ```bash
   ros2 param get /move_group use_sim_time
   ros2 param get /controller_manager use_sim_time
   ```

3. Verify controllers are active:
   ```bash
   ros2 control list_controllers
   ```

4. Check logs for errors:
   ```bash
   tail -50 /workspace/logs/gazebo_moveit_*.log
   ```

See [SETUP_GUIDE.md](SETUP_GUIDE.md#troubleshooting) for detailed troubleshooting.

## Credits

SO-ARM design with ROS2 Jazzy + MoveIt2 + Gazebo Harmonic integration.
