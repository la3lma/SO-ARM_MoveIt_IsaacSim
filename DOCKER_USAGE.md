# Docker Compose Usage Guide

This guide explains how to use the docker-compose setup for the SO-ARM MoveIt Isaac Sim project.

## Prerequisites

1. **Docker Desktop** installed and running
2. **X11 server** (for GUI applications like RViz):
   - **macOS**: Install [XQuartz](https://www.xquartz.org/)
   - **Linux**: X11 is typically pre-installed
   - **Windows**: Install [VcXsrv](https://sourceforge.net/projects/vcxsrv/) or Xming

## Quick Start

### 1. Setup Environment

Copy the example environment file:
```bash
cp .env.example .env
```

Edit `.env` if needed (especially `DISPLAY` variable for your OS).

### 2. For macOS with XQuartz

```bash
# Start XQuartz
open -a XQuartz

# Allow connections from localhost
xhost + localhost

# Set DISPLAY in .env file
echo "DISPLAY=host.docker.internal:0" >> .env
```

### 3. Basic Usage (Interactive Development)

Start the main container for interactive development:
```bash
docker-compose up ros2-moveit
```

This will:
- Build the workspace if needed
- Drop you into an interactive bash shell
- Provide available launch commands

Inside the container, you can run:
```bash
# Launch full MoveIt demo with fake controllers
ros2 launch so_arm_moveit_config demo.launch.py

# Or launch individual services
ros2 launch so_arm_moveit_config move_group.launch.py
ros2 launch so_arm_moveit_config moveit_rviz.launch.py
```

### 4. Full Stack (All Services)

Run all services together using the `full` profile:
```bash
docker-compose --profile full up
```

This starts:
- Robot State Publisher
- MoveIt Move Group (planning server)
- RViz (visualization)
- MongoDB (trajectory storage)
- Warehouse DB bridge

### 5. Visualization Only

If move group is already running, launch just RViz:
```bash
docker-compose --profile viz up rviz
```

### 6. Database Only

For motion planning storage:
```bash
docker-compose --profile database up mongodb warehouse
```

### 7. GPU Development (for Isaac Sim)

Use the GPU-enabled development container:
```bash
docker-compose --profile gpu up ros2-dev-gpu
```

**Note**: Requires NVIDIA GPU and nvidia-docker runtime.

## Service Profiles

The docker-compose file uses profiles to organize services:

| Profile | Services | Use Case |
|---------|----------|----------|
| (default) | ros2-moveit | Interactive development |
| `full` | All services | Complete system |
| `viz` | rviz | Visualization only |
| `database` | mongodb, warehouse | Motion planning storage |
| `gpu` | ros2-dev-gpu | GPU-accelerated development |
| `dev` | ros2-dev-gpu | Development with full tools |

## Common Commands

### Build/Rebuild Containers
```bash
# Build all images
docker-compose build

# Build specific service
docker-compose build ros2-moveit

# Force rebuild without cache
docker-compose build --no-cache
```

### Start Services
```bash
# Start in background
docker-compose up -d

# Start with specific profile
docker-compose --profile full up -d

# Start single service
docker-compose up ros2-moveit
```

### Stop Services
```bash
# Stop all running services
docker-compose down

# Stop and remove volumes
docker-compose down -v
```

### Access Running Containers
```bash
# Execute bash in running container
docker exec -it so-arm-moveit bash

# Run a specific command
docker exec -it so-arm-moveit ros2 topic list
```

### View Logs
```bash
# All services
docker-compose logs

# Specific service
docker-compose logs ros2-moveit

# Follow logs
docker-compose logs -f move-group
```

### Rebuild Workspace Inside Container
```bash
docker exec -it so-arm-moveit bash -c "cd /workspace && colcon build --symlink-install"
```

## Network Configuration

The compose file uses `network_mode: host` for ROS2 DDS communication. This allows:
- Zero-configuration multi-node communication
- Full compatibility with ROS2 discovery
- Access to host network services

**Alternative**: For isolated networks, you can modify docker-compose.yml to use bridge networking with proper DDS configuration.

## Volume Mounts

The following directories are mounted:
- `.:/workspace` - Your source code (read-write)
- `/tmp/.X11-unix` - X11 socket for GUI
- `~/.gitconfig` - Git configuration (read-only)
- `~/.ssh` - SSH keys for git operations (read-only)

## Troubleshooting

### RViz Won't Start (macOS)

1. Ensure XQuartz is running
2. Enable "Allow connections from network clients" in XQuartz preferences
3. Run: `xhost + localhost`
4. Set `DISPLAY=host.docker.internal:0` in `.env`

### DDS Communication Issues

If nodes can't discover each other:
1. Check `ROS_DOMAIN_ID` is consistent (default: 0)
2. Verify `network_mode: host` is set
3. Disable firewall temporarily to test

### Permission Denied Errors

If you get permission errors on files:
```bash
# Fix ownership (run on host)
sudo chown -R $USER:$USER .
```

### Build Failures

```bash
# Clean build
docker-compose down
rm -rf build install log
docker-compose build --no-cache
```

### MongoDB Connection Failed

```bash
# Check MongoDB is running
docker-compose ps mongodb

# View MongoDB logs
docker-compose logs mongodb

# Restart MongoDB
docker-compose restart mongodb
```

## Development Workflow

### 1. Code-Build-Test Cycle

```bash
# Terminal 1: Keep interactive container running
docker-compose up ros2-moveit

# Terminal 2: Make code changes in your editor

# Terminal 3: Rebuild inside container
docker exec -it so-arm-moveit bash -c "cd /workspace && colcon build --symlink-install --packages-select so_arm_moveit_config"

# Terminal 1: Test changes
ros2 launch so_arm_moveit_config demo.launch.py
```

### 2. Multi-Container Testing

```bash
# Start all services
docker-compose --profile full up

# In another terminal, interact with the system
docker exec -it so-arm-moveit bash
ros2 topic list
ros2 service list
ros2 node list
```

## Cleaning Up

### Remove Everything
```bash
# Stop all containers
docker-compose down

# Remove volumes
docker-compose down -v

# Remove images
docker rmi $(docker images 'so-arm*' -q)

# Clean build artifacts
rm -rf build install log
```

## Advanced: Custom Launch Commands

You can override the default command for any service:

```bash
docker-compose run --rm ros2-moveit bash -c "
  source /opt/ros/humble/setup.bash &&
  source install/setup.bash &&
  ros2 launch so_arm_moveit_config demo.launch.py
"
```

## Integration with Isaac Sim

For Isaac Sim integration (requires NVIDIA GPU):

1. Ensure nvidia-docker is installed
2. Use the GPU profile: `docker-compose --profile gpu up`
3. Install Isaac Sim separately or use NVIDIA's container
4. Configure ROS2 bridge in Isaac Sim to connect to the containers

## Support

For issues with:
- **Docker setup**: Check Docker Desktop logs
- **ROS2 issues**: Check container logs with `docker-compose logs`
- **MoveIt errors**: Enable MoveIt logging with `--log-level debug`

## Next Steps

- Customize launch files in `src/so_arm_moveit_config/launch/`
- Add custom controllers in `src/so_arm_controllers/`
- Integrate hardware interface in `src/so_arm_hardware_interface/`
- Connect to Isaac Sim for realistic simulation
