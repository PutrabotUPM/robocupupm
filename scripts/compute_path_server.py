#!/usr/bin/env python
import rospy
from your_package.srv import ComputePath, ComputePathResponse
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

def handle_compute_path(req):
    rospy.loginfo("Received path request from (%f, %f) to (%f, %f)",
                  req.start.pose.position.x, req.start.pose.position.y,
                  req.goal.pose.position.x, req.goal.pose.position.y)

    path = Path()
    path.header.frame_id = "map"
    path.header.stamp = rospy.Time.now()

    # Example path (straight line)
    for i in range(11):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = req.start.pose.position.x + (req.goal.pose.position.x - req.start.pose.position.x) * i / 10.0
        pose.pose.position.y = req.start.pose.position.y + (req.goal.pose.position.y - req.start.pose.position.y) * i / 10.0
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)

    return ComputePathResponse(path)

def compute_path_server():
    rospy.init_node('compute_path_server')
    rospy.Service('compute_path', ComputePath, handle_compute_path)
    rospy.loginfo("Ready to compute paths.")
    rospy.spin()

if __name__ == "__main__":
    compute_path_server()

