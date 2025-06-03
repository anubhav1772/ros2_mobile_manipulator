# ros2_mobile_manipulator
A ROS 2-based mobile manipulator platform.

Ridgeback + UR5 (ROS 2 Humble and Gazebo/Ignition)

### Steps
Start the simulation using:

    $ ros2 launch ridgeback_ur5_gazebo bringup.launch.py
    
Move the joints (sends a goal to the `/manipulator_controller/follow_joint_trajectory` action server):

    $ ros2 action send_goal /manipulator_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: ['ur_arm_shoulder_pan_joint', 'ur_arm_shoulder_lift_joint', 'ur_arm_elbow_joint', 'ur_arm_wrist_1_joint', 'ur_arm_wrist_2_joint', 'ur_arm_wrist_3_joint'], points: [{positions: [0.0, -0.5, 0.5, 0.0, 1.0, 0.0], time_from_start: {sec: 2}}]}}"


