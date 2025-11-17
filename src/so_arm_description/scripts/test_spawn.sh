#!/bin/bash
set -x

echo "=== Testing Gazebo Spawn ==="

# Check if Gazebo is running
echo "1. Checking Gazebo process..."
pgrep -a gz

# Check if /robot_description topic exists
echo "2. Checking /robot_description topic..."
ros2 topic info /robot_description

# Try to echo the topic content
echo "3. Getting robot_description content..."
timeout 2 ros2 topic echo /robot_description --once

# List all models in Gazebo
echo "4. Listing models in Gazebo..."
gz model --list

# Check Gazebo service for spawning
echo "5. Checking Gazebo services..."
gz service --list | grep -i create

# Try manual spawn with gz command
echo "6. Attempting manual spawn..."
gz service -s /world/empty/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 5000 \
  --req "sdf_filename: '/workspace/install/so_arm_urdf/share/so_arm_urdf/urdf/so_arm_urdf.urdf', name: 'test_robot'"

echo "7. Listing models again..."
gz model --list
