# SO-ARM MoveIt IsaacSim

A ROS 2 Humble workspace for the SO-ARM robot with MoveIt motion planning integration for Isaac Sim.

## Overview

This project provides MoveIt configuration and URDF description for the SO-ARM robot, designed to work with NVIDIA Isaac Sim for robotics simulation and motion planning.

## Features

- **ROS 2 Humble** compatibility
- **MoveIt 2** motion planning configuration
- **Docker** containerization for easy deployment
- **Isaac Sim** integration ready
- Complete URDF robot description
- Pre-configured launch files

## Project Structure

```
so-arm_moveit_isaacsim/
├── src/
│   ├── so_arm_urdf/              # Robot URDF description
│   └── so_arm_moveit_config/     # MoveIt configuration
├── Dockerfile                    # Full Docker setup
├── Dockerfile.minimal           # Minimal Docker setup
├── build.log                    # Build logs
├── deps.txt                     # Dependencies list
└── find_deps.sh                 # Dependency finder script
```

## Quick Start

### Option 1: Docker (Recommended)

1. **Clone the repository:**
   ```bash
   git clone https://github.com/MuammerBay/SO-ARM_MoveIt_IsaacSim.git
   cd SO-ARM_MoveIt_IsaacSim
   ```

2. **Build the Docker image:**
   ```bash
   docker build -f Dockerfile.minimal -t so-arm-minimal .
   ```

3. **Run the container:**
   ```bash
   docker run -it --rm so-arm-minimal bash
   ```

4. **Inside the container, test the packages:**
   ```bash
   source /opt/ros/humble/setup.sh
   source install/setup.sh
   ros2 pkg list | grep so_arm
   ```

### Option 2: Local Build

1. **Prerequisites:**
   - ROS 2 Humble installed
   - MoveIt 2 packages
   - colcon build tools

2. **Clone and build:**
   ```bash
   git clone https://github.com/MuammerBay/SO-ARM_MoveIt_IsaacSim.git
   cd SO-ARM_MoveIt_IsaacSim
   colcon build
   source install/setup.bash
   ```

## Usage

### Launch MoveIt Demo

```bash
# Source the workspace
source install/setup.bash

# Launch MoveIt demo (when available)
ros2 launch so_arm_moveit_config demo.launch.py
```

### Available Packages

- **so_arm_urdf**: Robot description package with URDF files
- **so_arm_moveit_config**: MoveIt configuration with planning scenes, controllers, and launch files

## Docker Images

### Minimal Image (`Dockerfile.minimal`)
- Based on `ros:humble-ros-base`
- Includes core MoveIt packages
- Optimized for CI/CD and basic testing
- ~2GB compressed

### Full Image (`Dockerfile`)
- Includes additional development tools
- GUI support for RViz
- Complete development environment

## Development

### Building Locally

```bash
# Clean previous builds (if folder was renamed)
rm -rf build/ install/ log/

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash
```

### Adding New Packages

1. Add your package to the `src/` directory
2. Update dependencies in `package.xml`
3. Rebuild: `colcon build`

## Troubleshooting

### Folder Name Change Issues

If you renamed the workspace folder, clean the build artifacts:

```bash
rm -rf build/ install/ log/
colcon build
```

### Docker Build Issues

If Docker build fails due to missing packages:

```bash
# Use the minimal Dockerfile which skips problematic dependencies
docker build -f Dockerfile.minimal -t so-arm-minimal .
```

### Missing Dependencies

Check available packages:

```bash
# List ROS packages
apt list | grep ros-humble-

# Check rosdep dependencies
rosdep install --from-paths src --ignore-src -r -y --simulate
```

## Contributing

1. Fork the repository
2. Create a feature branch: `git checkout -b feature-name`
3. Commit changes: `git commit -am 'Add feature'`
4. Push to branch: `git push origin feature-name`
5. Submit a Pull Request

## License

This project is licensed under the BSD License - see the LICENSE file for details.

## Acknowledgments

- Built with ROS 2 Humble and MoveIt 2
- Designed for NVIDIA Isaac Sim integration
- Based on standard ROS robotics practices

## Contact

For questions and support, please open an issue on GitHub. 