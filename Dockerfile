FROM ros:humble

# --- Unminimize Ubuntu (restore man pages, docs, etc.) ---
RUN yes | unminimize || true

# Add Gazebo repository
RUN apt-get update && apt-get install -y wget gnupg lsb-release \
    && wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list

# Install dependencies including desktop packages, VNC, SSH, GUI tools, and Gazebo
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
    ros-humble-rviz-default-plugins \
    ros-humble-ros-gz \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-bridge \
    ros-humble-gz-ros2-control \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-joint-trajectory-controller \
    ros-humble-position-controllers \
    gz-garden \
    mesa-utils libgl1 libgl1-mesa-dev libglu1-mesa-dev \
    libxkbcommon-x11-0 libxrandr2 libxi6 libxrender1 libxss1 \
    libxtst6 libxcomposite1 libasound2 \
    novnc websockify tigervnc-standalone-server \
    xfce4 xfce4-terminal terminator dbus-x11 \
    sudo wget gpg ca-certificates \
    openssh-server \
    emacs \
    less \
    && rm -rf /var/lib/apt/lists/* \
    && mkdir -p /run/sshd

# --- Non-root user (nicer dev UX, writable mounts) ---
ARG USER=dev
ARG UID=1000
ARG GID=1000
ARG USER_PASSWORD=moveit
# Handle potentially existing user/group from base image
RUN set -x \
 && EXISTING_USER=$(getent passwd ${UID} | cut -d: -f1 || true) \
 && EXISTING_GROUP=$(getent group ${GID} | cut -d: -f1 || true) \
 && if [ -n "$EXISTING_USER" ] && [ "$EXISTING_USER" != "${USER}" ]; then \
      userdel -r $EXISTING_USER || true; \
    fi \
 && if [ -n "$EXISTING_GROUP" ] && [ "$EXISTING_GROUP" != "${USER}" ]; then \
      groupdel $EXISTING_GROUP || true; \
    fi \
 && groupadd -g ${GID} ${USER} || true \
 && useradd -m -u ${UID} -g ${GID} -s /bin/bash ${USER} \
 && echo "${USER}:${USER_PASSWORD}" | chpasswd \
 && echo "${USER} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USER} \
 && chmod 0440 /etc/sudoers.d/${USER}

# --- SSH setup for the dev user ---
RUN mkdir -p /home/${USER}/.ssh \
 && chmod 700 /home/${USER}/.ssh \
 && touch /home/${USER}/.ssh/authorized_keys \
 && chmod 600 /home/${USER}/.ssh/authorized_keys \
 && chown -R ${USER}:${USER} /home/${USER}/.ssh

# Copy source code
COPY src /workspace/src

WORKDIR /workspace

# Install ROS dependencies (skip warehouse_ros_mongo as it's not available in Humble)
RUN rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y --skip-keys="warehouse_ros_mongo"

# Build the workspace
RUN . /opt/ros/humble/setup.sh && \
    colcon build

USER ${USER}

# --- VNC setup (password + XFCE startup) ---
ARG VNC_PASSWORD=moveit
RUN mkdir -p /home/${USER}/.vnc \
 && printf "%s\n%s\n\n" "${VNC_PASSWORD}" "${VNC_PASSWORD}" | vncpasswd -f > /home/${USER}/.vnc/passwd \
 && chmod 600 /home/${USER}/.vnc/passwd

# xstartup: launch XFCE
RUN cat > /home/${USER}/.vnc/xstartup <<'EOF' && \
    chmod +x /home/${USER}/.vnc/xstartup
#!/bin/sh
unset SESSION_MANAGER
unset DBUS_SESSION_BUS_ADDRESS
export XDG_RUNTIME_DIR=/tmp/runtime-dev
mkdir -p "$XDG_RUNTIME_DIR" && chmod 700 "$XDG_RUNTIME_DIR"
if command -v startxfce4 >/dev/null 2>&1; then
  exec startxfce4
else
  exec xterm
fi
EOF

# Default VNC config: allow external (via Docker port), set sane geometry/depth
RUN cat > /home/${USER}/.vnc/config <<'EOF'
localhost=no
geometry=1920x1080
depth=24
EOF

# Make sure ROS env is available in shells launched from XFCE terminal
RUN echo "source /opt/ros/humble/setup.bash" >> /home/${USER}/.bashrc && \
    echo '[ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash' >> /home/${USER}/.bashrc

# --- Setup /etc/profile.d/ for login shells (SSH sessions) ---
USER root
RUN cat > /etc/profile.d/ros2-setup.sh <<'EOF'
#!/bin/bash
# Source ROS 2 base installation
source /opt/ros/humble/setup.bash
# Source workspace overlay if it exists
[ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash
EOF
RUN chmod +x /etc/profile.d/ros2-setup.sh

# --- Entrypoint to start SSH + VNC + noVNC ---
RUN printf '%s\n' '#!/usr/bin/env bash' \
  'set -e' \
  '' \
  '# Start SSH server' \
  '/usr/sbin/sshd' \
  '' \
  '# Setup VNC' \
  'export DISPLAY=:1' \
  'export XDG_RUNTIME_DIR=/tmp/runtime-'${USER} \
  'mkdir -p "$XDG_RUNTIME_DIR" && chmod 700 "$XDG_RUNTIME_DIR" && chown -R '${USER}:${USER}' "$XDG_RUNTIME_DIR"' \
  'GEOM="${VNC_GEOMETRY:-1920x1080}"' \
  'DEPTH="${VNC_DEPTH:-24}"' \
  'su - '${USER}' -c "vncserver :1 -geometry ${GEOM} -depth ${DEPTH} -localhost no"' \
  '' \
  '# Start noVNC (this blocks, so it must be last)' \
  'if [ -f /usr/share/novnc/utils/novnc_proxy ]; then' \
  '  exec /usr/share/novnc/utils/novnc_proxy --vnc localhost:5901 --listen 6080' \
  'elif [ -f /usr/share/novnc/utils/launch.sh ]; then' \
  '  exec /usr/share/novnc/utils/launch.sh --vnc localhost:5901 --listen 6080' \
  'else' \
  '  exec websockify --web /usr/share/novnc 6080 localhost:5901' \
  'fi' \
  > /usr/local/bin/start-services.sh \
  && chmod +x /usr/local/bin/start-services.sh

EXPOSE 22 5901 6080
CMD ["/usr/local/bin/start-services.sh"] 