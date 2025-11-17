#!/usr/bin/bash 


positions=( "[0.0, 0.5, -0.5, 0.3, 0.0, 0.0]"
	    "[0.0, 0.5,  0.5, 0.3, 0.0, 0.0]"
	    "[0.0, 0.5, -0.5, 0.3, 0.5, 0.5]" 	    
	  )

while /bin/true; do 
  for pos in "${positions[@]}"; do
      ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{    joint_names: ['Rotation', 'Pitch', 'Elbow', 'Wrist_Pitch', 'Wrist_Roll', 'Jaw'],   points: [ {positions: $pos, time_from_start: {sec: 3}} ] }"
      sleep 5
  done   
done
