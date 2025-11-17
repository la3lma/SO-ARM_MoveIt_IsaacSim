#!/usr/bin/env python3
"""
Manually spawn SO-ARM robot in Gazebo with proper mesh paths
"""
import os
import sys
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import subprocess
import time

def main():
    # Get package path
    pkg_path = get_package_share_directory('so_arm_urdf')
    urdf_file = os.path.join(pkg_path, 'urdf', 'so_arm_urdf.urdf')
    mesh_dir = os.path.join(pkg_path, 'meshes')

    print(f"Package path: {pkg_path}")
    print(f"URDF file: {urdf_file}")
    print(f"Mesh directory: {mesh_dir}")

    # Read URDF
    with open(urdf_file, 'r') as f:
        urdf_content = f.read()

    # Replace package:// URIs with file:// URIs for Gazebo
    urdf_content = urdf_content.replace(
        'package://so_arm_urdf/meshes/',
        f'file://{mesh_dir}/'
    )

    # Save modified URDF
    temp_urdf = '/tmp/so_arm_gazebo.urdf'
    with open(temp_urdf, 'w') as f:
        f.write(urdf_content)

    print(f"Created temporary URDF with file:// paths: {temp_urdf}")

    # Convert to SDF
    print("Converting URDF to SDF...")
    result = subprocess.run(
        ['gz', 'sdf', '-p', temp_urdf],
        capture_output=True,
        text=True
    )

    if result.returncode != 0:
        print(f"Error converting URDF to SDF: {result.stderr}")
        return 1

    sdf_content = result.stdout
    temp_sdf = '/tmp/so_arm_gazebo.sdf'
    with open(temp_sdf, 'w') as f:
        f.write(sdf_content)

    print(f"Created SDF file: {temp_sdf}")

    # Wait a bit to ensure Gazebo is ready
    print("Waiting for Gazebo to be ready...")
    time.sleep(2)

    # Spawn using gz service
    print("Spawning robot in Gazebo...")
    spawn_cmd = [
        'gz', 'service',
        '-s', '/world/empty/create',
        '--reqtype', 'gz.msgs.EntityFactory',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', '5000',
        '--req', f'sdf_filename: "{temp_sdf}", name: "so_arm", pose: {{position: {{x: 0, y: 0, z: 0.2}}}}'
    ]

    result = subprocess.run(spawn_cmd, capture_output=True, text=True)
    print(f"Spawn result: {result.stdout}")
    if result.stderr:
        print(f"Spawn errors: {result.stderr}")

    return 0 if result.returncode == 0 else 1

if __name__ == '__main__':
    sys.exit(main())
