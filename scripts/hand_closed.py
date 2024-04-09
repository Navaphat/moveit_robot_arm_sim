#! /usr/bin/env python3

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

def move_hand(joint_positions):

    rospy.init_node('hand_closed')

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
        joint_hand_positions = [0.0011578212707099127, -0.008221930186906512]
        move_hand(joint_hand_positions)
        rospy.loginfo("Complete")
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted")