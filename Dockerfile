FROM ros:humble

# Install dependencies including desktop packages
RUN apt-get update && apt-get install -y \
    python3-rosdep \
    python3-colcon-common-extensions \
    ros-humble-desktop \
    ros-humble-moveit \
    ros-humble-controller-manager \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# Copy source code
COPY src /workspace/src

WORKDIR /workspace

# Install ROS dependencies (skip warehouse_ros_mongo as it's not available in Humble)
RUN rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y --skip-keys="warehouse_ros_mongo"

# Build the workspace
RUN . /opt/ros/humble/setup.sh && \
    colcon build

# Setup environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /workspace/install/setup.bash" >> ~/.bashrc

# Default command
CMD ["bash"] 