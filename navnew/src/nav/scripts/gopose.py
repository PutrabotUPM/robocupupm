#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import PoseStamped

def get_robot_pose():
    # Initialize the ROS node
    rospy.init_node('get_robot_pose', anonymous=True)

    # Create a TF listener
    listener = tf.TransformListener()

    # Create a publisher for PoseStamped messages
    pose_pub = rospy.Publisher('/robot_pose', PoseStamped, queue_size=10)

    try:
        # Wait for the transform between 'map' and 'base_link'
        listener.waitForTransform('/map', '/base_link', rospy.Time(0), rospy.Duration(5.0))
        (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))

        # Create a PoseStamped message
        pose_msg = PoseStamped()

        # Set the header
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = 'map'

        # Set the position (translation)
        pose_msg.pose.position.x = trans[0]
        pose_msg.pose.position.y = trans[1]
        pose_msg.pose.position.z = trans[2]

        # Set the orientation (rotation)
        pose_msg.pose.orientation.x = rot[0]
        pose_msg.pose.orientation.y = rot[1]
        pose_msg.pose.orientation.z = rot[2]
        pose_msg.pose.orientation.w = rot[3]

        # Publish the pose message
        pose_pub.publish(pose_msg)

        rospy.loginfo("Published robot pose: {}".format(pose_msg))

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logwarn("Could not find the transform from 'map' to 'base_link'.")

if __name__ == '__main__':
    try:
        get_robot_pose()
    except rospy.ROSInterruptException:
        pass
