# Docker Compose Usage Guide

This guide explains how to use the docker-compose setup for the SO-ARM MoveIt Isaac Sim project with SSH and VNC support.

## Features

- **SSH Access** with X11 forwarding for remote development
- **VNC Desktop** with XFCE4 for full GUI experience
- **noVNC** web-based VNC access (no client needed)
- **ROS2 Humble** with MoveIt integration
- **Auto-configured** environment in all access methods

## Prerequisites

1. **Docker Desktop** installed and running
2. **SSH client** (built-in on macOS/Linux)
3. **Web browser** for noVNC access (optional)
4. **VNC client** (optional, for direct VNC access)

## Quick Start

### 1. Build and Start

```bash
# Using Makefile (recommended)
make up

# Or using docker-compose directly
docker compose up -d
```

This will:
- Build the Docker image with SSH and VNC support
- Start the container in background
- Expose ports for SSH (2222), VNC (5901), and noVNC (6080)

### 2. Setup SSH Keys (One-time)

```bash
make ssh-setup
```

This copies your SSH public key from `~/.ssh/` into the container for passwordless SSH access.

### 3. Access Methods

You now have three ways to access your ROS2 environment:

#### Option A: SSH with X11 Forwarding (Recommended for Development)

```bash
# Using the helper script
make ssh

# Or directly
./robossh

# Or manually
ssh -X -p 2224 dev@127.0.0.1
```

**Benefits:**
- Fast and lightweight
- X11 forwarding allows GUI apps (RViz) to display on your host
- Full terminal access with ROS environment pre-configured
- Ideal for CLI-heavy workflows

**Password:** `moveit` (if you didn't setup SSH keys)

#### Option B: noVNC (Web-based Desktop)

```bash
# Open in browser automatically
make vnc

# Or manually open
open http://localhost:6082/vnc.html

# Start container: `make up`
2. Open noVNC: `make vnc` or go to http://localhost:6082/vnc.html
```

**Benefits:**
- No client installation needed (uses web browser)
- Full XFCE desktop environment
- Can run multiple GUI applications simultaneously
- Great for demos and visual work

**Password:** `moveit`

#### Option C: VNC Client (Full Desktop)

Use any VNC client and connect to:
- **Host:** `localhost`
- **Port:** `5903`
- **Password:** `moveit`

**macOS:** Built-in Screen Sharing app
```bash
open vnc://localhost:5903
```

**Linux:** Use Remmina, TigerVNC, or similar
```bash
vncviewer localhost:5903
```

## Common Workflows

### Development with SSH + X11

```bash
# SSH into container
make ssh

# Inside the container, ROS environment is already sourced
# Launch RViz directly (displays on your host)
ros2 launch so_arm_moveit_config demo.launch.py

# Edit files on host with your favorite editor
# Changes are immediately visible in the container
```

### Full Desktop Experience with VNC

1. Start container: `make up`
2. Open noVNC: `make vnc` or go to http://localhost:6080/vnc.html
3. You'll see a full XFCE desktop
4. Open a terminal in XFCE
5. Run ROS commands:
   ```bash
   ros2 launch so_arm_moveit_config demo.launch.py
   ```

### IDE Integration (CLion, VSCode)

**VSCode Remote SSH:**
1. Install "Remote - SSH" extension
2. Add SSH target: `ssh://dev@127.0.0.1:2224`
3. Connect and open `/workspace`
4. ROS environment auto-loads in terminal

**CLion:**
1. Go to File → Settings → Build, Execution, Deployment → Toolchains
2. Add Remote Host
3. Set SSH: `dev@127.0.0.1:2224`
4. CMake will auto-detect ROS

## Makefile Commands

```bash
make help       # Show all available commands
make build      # Build Docker image
make up         # Start container
make down       # Stop container
make restart    # Restart container
make ssh-setup  # Setup SSH keys (one-time)
make ssh        # SSH into container with X11
make vnc        # Open noVNC in browser
make logs       # View container logs
make rebuild    # Rebuild from scratch (no cache)
make clean      # Remove container and image
```

## Environment Configuration

Copy and customize the environment file:

```bash
cp .env.example .env
```

Edit `.env` to customize:
- `VNC_GEOMETRY`: Desktop resolution (default: 1920x1080)
- `VNC_DEPTH`: Color depth (default: 24)
- `ROS_DOMAIN_ID`: ROS2 DDS domain (default: 0)
- `TZ`: Timezone (default: UTC)

## Ports

All ports are bound to `127.0.0.1` (localhost only) for security:

| Port | Service | Access |
|------|---------|--------|
| 2224 | SSH | `ssh -p 2224 dev@127.0.0.1` |
| 5903 | VNC | `vnc://localhost:5903` |
| 6082 | noVNC | http://localhost:6082/vnc.html |

## Credentials

- **User:** `dev`
- **SSH Password:** `moveit` (or use SSH keys)
- **VNC Password:** `moveit`
- **Sudo:** Passwordless sudo enabled

## Running ROS2 Applications

All access methods have the ROS2 environment pre-configured. Simply run:

```bash
# Launch MoveIt demo with fake controllers
ros2 launch so_arm_moveit_config demo.launch.py

# Launch move_group planning server
ros2 launch so_arm_moveit_config move_group.launch.py

# Launch RViz with MoveIt plugin
ros2 launch so_arm_moveit_config moveit_rviz.launch.py

# List available topics
ros2 topic list

# Check node status
ros2 node list
```

## Troubleshooting

### SSH Connection Refused

```bash
# Check if container is running
docker ps | grep so-arm-moveit

# Check SSH service inside container
docker exec so-arm-moveit ps aux | grep sshd

# View container logs
make logs
```

### VNC Not Connecting

```bash
# Check VNC service
docker exec so-arm-moveit ps aux | grep vnc

# Restart container
make restart

# Check port mappings
docker port so-arm-moveit
```

### X11 Forwarding Not Working (macOS)

```bash
# Install XQuartz
brew install --cask xquartz

# Enable X11 forwarding in XQuartz Preferences:
# Security → "Allow connections from network clients"

# Restart XQuartz and allow connections
xhost + localhost
```

### ROS Environment Not Sourced

The environment should auto-load, but if not:

```bash
# In SSH session
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash

# In VNC terminal (already in .bashrc)
source ~/.bashrc
```

### Permission Issues

```bash
# Fix workspace ownership (run on host)
sudo chown -R $USER:$USER .

# Or set matching UID in Dockerfile
docker compose build --build-arg UID=$(id -u) --build-arg GID=$(id -g)
```

## Advanced Usage

### Custom VNC Password

Edit Dockerfile and rebuild:
```dockerfile
ARG VNC_PASSWORD=your_password
```

### Persist VNC Settings

Uncomment in docker-compose.yml:
```yaml
volumes:
  - ./vnc:/home/dev/.vnc
```

### Change Desktop Resolution

```bash
# Set in .env file
VNC_GEOMETRY=2560x1440

# Or at runtime
docker compose up -d -e VNC_GEOMETRY=2560x1440
```

### Multiple Concurrent Users

Create additional service definitions in docker-compose.yml with different port mappings.

### Connect from Another Machine

⚠️ **Security Warning:** Only do this on trusted networks!

Change port bindings in docker-compose.yml:
```yaml
ports:
  - "0.0.0.0:6082:6080"  # Expose on all interfaces
  - "0.0.0.0:2224:22"
```

## Development Workflow Examples

### Example 1: Quick Testing with SSH

```bash
# Terminal 1: SSH into container
make ssh

# Inside container
ros2 launch so_arm_moveit_config demo.launch.py

# RViz appears on your host screen
# Terminal 2: Edit code on host
vim src/so_arm_moveit_config/config/kinematics.yaml

# Changes reflected immediately (volume mount)
```

### Example 2: Full Desktop Development

```bash
# Start VNC desktop
make vnc

# In browser VNC session:
# 1. Open Terminator terminal
# 2. Split into multiple panes (Ctrl+Shift+E, Ctrl+Shift+O)
# 3. In pane 1: ros2 launch so_arm_moveit_config move_group.launch.py
# 4. In pane 2: ros2 launch so_arm_moveit_config moveit_rviz.launch.py
# 5. In pane 3: ros2 topic echo /joint_states
```

### Example 3: IDE + Container Development

```bash
# VSCode
1. Install Remote-SSH extension
2. Connect to dev@127.0.0.1:2222
3. Open folder: /workspace
4. Use integrated terminal for ROS commands
5. Edit, build, run - all in VSCode

# CLion
1. Configure Remote Toolchain (127.0.0.1:2222)
2. Load /workspace/src as CMake project
3. Run/Debug ROS nodes directly from IDE
```

## Integration with Other Services

### MongoDB for Trajectory Storage

Uncomment in docker-compose.yml:
```yaml
services:
  mongodb:
    ...
  warehouse:
    ...
```

Then enable in launch files or connect manually.

### Isaac Sim Integration

For Isaac Sim, you'll need GPU passthrough. See the GPU profile in docker-compose.yml and ensure nvidia-docker is installed.

## Cleaning Up

```bash
# Stop and remove containers
make down

# Remove everything including images
make clean

# Clean build artifacts
rm -rf build install log
```

## Next Steps

- Customize URDF models in `src/so_arm_description/`
- Modify MoveIt config in `src/so_arm_moveit_config/config/`
- Add custom controllers in `src/so_arm_controllers/`
- Integrate hardware interface in `src/so_arm_hardware_interface/`
- Connect to real robot or Isaac Sim

## Support

For issues:
- Check container logs: `make logs`
- Rebuild image: `make rebuild`
- Verify ports: `docker port so-arm-moveit`
- Test SSH: `ssh -v -p 2224 dev@127.0.0.1`

## Summary of Access Methods

| Method | Use Case | Command | GUI Support |
|--------|----------|---------|-------------|
| SSH + X11 | Development, CLI-heavy | `make ssh` | Single app (RViz) |
| noVNC | Demos, full desktop | `make vnc` | Full desktop |
| VNC Client | Power users | VNC client | Full desktop |
| Docker exec | Quick debugging | `docker exec -it so-arm-moveit bash` | No |

Choose the method that fits your workflow!
