#!/bin/bash
# Wait for nodes to start
echo "Waiting for nodes to start..."
sleep 15

# Set use_sim_time parameter for move_group
echo "Setting use_sim_time=true for all nodes..."
ros2 param set /move_group use_sim_time true
ros2 param set /controller_manager use_sim_time true 2>/dev/null || true
ros2 param set /arm_controller use_sim_time true 2>/dev/null || true
ros2 param set /joint_state_broadcaster use_sim_time true 2>/dev/null || true
ros2 param set /robot_state_publisher use_sim_time true 2>/dev/null || true

echo "use_sim_time parameters set!"
echo "move_group: $(ros2 param get /move_group use_sim_time)"
echo "controller_manager: $(ros2 param get /controller_manager use_sim_time 2>&1 | head -1)"
