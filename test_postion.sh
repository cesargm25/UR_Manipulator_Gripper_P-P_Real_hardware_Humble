ros2 topic pub /scaled_joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
joint_names:
  - shoulder_pan_joint
  - shoulder_lift_joint
  - elbow_joint
  - wrist_1_joint
  - wrist_2_joint
  - wrist_3_joint
points:
  - positions:
      - 1.57
      - -1.57
      - 1.57
      - 0.0
      - 1.57
      - 0.0
    velocities: []
    accelerations: []
    effort: []
    time_from_start:
      sec: 3
      nanosec: 0"
