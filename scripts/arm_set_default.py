#! /usr/bin/env python3

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

def move_arm(joint_positions):
    # Initialize ROS node
    rospy.init_node('set_default')

    # Connect to the arm controller action server
    arm_client = actionlib.SimpleActionClient('/robot_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    arm_client.wait_for_server()

    # Create a trajectory goal
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']  # Modify joint names as per your robot

    # Create a trajectory point with the desired joint positions
    point = JointTrajectoryPoint()
    point.positions = joint_positions
    point.time_from_start = rospy.Duration(2)  # Specify time for the arm to reach the goal positions

    # Add the trajectory point to the goal
    goal.trajectory.points.append(point)

    # Send the goal to the arm controller
    arm_client.send_goal(goal)

    # Wait for the arm to reach the goal
    arm_client.wait_for_result()

def move_hand(joint_positions):

    # Connect to the arm controller action server
    hand_client = actionlib.SimpleActionClient('/hand_ee_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    hand_client.wait_for_server()

    # Create a trajectory goal
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = ['joint_6', 'joint_7']  # Modify joint names as per your robot

    # Create a trajectory point with the desired joint positions
    point = JointTrajectoryPoint()
    point.positions = joint_positions
    point.time_from_start = rospy.Duration(2)  # Specify time for the arm to reach the goal positions

    # Add the trajectory point to the goal
    goal.trajectory.points.append(point)

    # Send the goal to the arm controller
    hand_client.send_goal(goal)

    # Wait for the arm to reach the goal
    hand_client.wait_for_result()

if __name__ == '__main__':
    try:
        # Example joint positions (modify as per your robot's configuration)
        joint_arm_positions = [2.7920025819912554e-05, 1.5699069995144617, -6.303337262943386e-05, -3.1419243230744263, 4.87741849385202e-05]
        joint_hand_positions = [0.029901434279512612, -0.029919463811442254]
        move_arm(joint_arm_positions)
        move_hand(joint_hand_positions)
        rospy.loginfo("Complete")
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted")