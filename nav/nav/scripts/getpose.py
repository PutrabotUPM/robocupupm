#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
import tf.transformations as tft

def movebase_client(x, y, yaw):
    # Create an action client for the move_base action server
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    # Wait for the action server to be available
    rospy.loginfo("Waiting for move_base action server...")
    client.wait_for_server()

    # Create a new goal to send to move_base
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # Set the goal's position and orientation
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = 0.0

    # Convert yaw to a quaternion and set the orientation
    quat = tft.quaternion_from_euler(0.0, 0.0, yaw)
    goal.target_pose.pose.orientation.x = quat[0]
    goal.target_pose.pose.orientation.y = quat[1]
    goal.target_pose.pose.orientation.z = quat[2]
    goal.target_pose.pose.orientation.w = quat[3]

    # Send the goal to move_base
    rospy.loginfo(f"Sending goal: Position({x}, {y}), Orientation(yaw: {yaw})")
    client.send_goal(goal)

    # Wait for the result
    client.wait_for_result()

    # Check if the goal was achieved
    if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Goal reached successfully!")
    else:
        rospy.logwarn("Failed to reach goal. The robot might be stuck or the goal is unreachable.")

def pose_callback(pose_msg):
    # Extract position and orientation from the received Float32MultiArray message
    x = pose_msg.data[0]
    y = pose_msg.data[1]
    yaw = pose_msg.data[2]

    # Call the movebase_client function with the received coordinates
    movebase_client(x, y, yaw)

if __name__ == '__main__':
    try:
        # Initialize the ROS node
        rospy.init_node('move_base_client_node')

        # Subscribe to the get_robot_pose topic that provides the robot's target pose
        rospy.Subscriber('/get_robot_pose', Float32MultiArray, pose_callback)

        # Keep the node running
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")
