FROM ros:humble-desktop

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-rosdep \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Copy source code
COPY src /workspace/src

WORKDIR /workspace

# Install ROS dependencies
RUN rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
RUN . /opt/ros/humble/setup.sh && \
    colcon build

# Setup environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /workspace/install/setup.bash" >> ~/.bashrc

# Default command
CMD ["bash"] 