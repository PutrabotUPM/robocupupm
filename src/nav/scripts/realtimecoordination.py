#!/usr/bin/env python

import rospy
import tf

def get_robot_pose():
    # Initialize the ROS node
    rospy.init_node('get_robot_pose', anonymous=True)

    # Create a TF listener
    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)  # Rate to check for transformations

    while not rospy.is_shutdown():
        try:
            # Wait for the transform between 'map' and 'base_link'
            (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))

            # Extract position (trans)
            x, y, z = trans

            # Extract orientation (rot)
            x_q, y_q, z_q, w_q = rot

            # Log the position and orientation
            rospy.loginfo("Position -> x: {:.2f}, y: {:.2f}, z: {:.2f}".format(x, y, z))
            rospy.loginfo("Orientation (Quaternion) -> x: {:.2f}, y: {:.2f}, z: {:.2f}, w: {:.2f}".format(x_q, y_q, z_q, w_q))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Could not find the transform from 'map' to 'base_link'. Retrying...")

        rate.sleep()

if __name__ == '__main__':
    try:
        get_robot_pose()
    except rospy.ROSInterruptException:
        pass