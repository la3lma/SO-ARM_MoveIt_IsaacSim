#!/bin/bash
# Launch Gazebo + MoveIt and log all output with timestamps
# Usage: ./launch_and_log.sh

TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
LOG_DIR="/workspace/logs"
LOG_FILE="${LOG_DIR}/gazebo_moveit_${TIMESTAMP}.log"

# Create logs directory if it doesn't exist
mkdir -p "${LOG_DIR}"

echo "========================================" | tee "${LOG_FILE}"
echo "Gazebo + MoveIt Launch Log" | tee -a "${LOG_FILE}"
echo "Started at: $(date)" | tee -a "${LOG_FILE}"
echo "Log file: ${LOG_FILE}" | tee -a "${LOG_FILE}"
echo "========================================" | tee -a "${LOG_FILE}"
echo "" | tee -a "${LOG_FILE}"

# Source ROS environment
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash

# Launch with output redirection
echo "Launching ros2 launch so_arm_moveit_config gazebo_moveit.launch.py..." | tee -a "${LOG_FILE}"
echo "" | tee -a "${LOG_FILE}"

ros2 launch so_arm_moveit_config gazebo_moveit.launch.py 2>&1 | tee -a "${LOG_FILE}"

# Capture exit status
EXIT_STATUS=$?

echo "" | tee -a "${LOG_FILE}"
echo "========================================" | tee -a "${LOG_FILE}"
echo "Launch terminated at: $(date)" | tee -a "${LOG_FILE}"
echo "Exit status: ${EXIT_STATUS}" | tee -a "${LOG_FILE}"
echo "========================================" | tee -a "${LOG_FILE}"

exit ${EXIT_STATUS}
