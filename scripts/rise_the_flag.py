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

if __name__ == '__main__':
    try:
        # Example joint positions (modify as per your robot's configuration)
        joint_arm_positions = [4.935628320090473e-05, 8.008044278249144e-05, -1.5720182332717814, -1.5720713824735955, 5.873241457156837e-05]
        move_arm(joint_arm_positions)
        rospy.loginfo("Complete")
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted")