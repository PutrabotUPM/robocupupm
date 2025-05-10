#!/usr/bin/env python3

# import rospy
# from nav_msgs.msg import Odometry
# import os

# def odom_callback(msg):
#     initial_pose = msg.pose.pose
#     with open("/home/neow/initial_pose.txt", "w") as file:
#         file.write(f"{initial_pose.position.x} {initial_pose.position.y} "
#                    f"{initial_pose.orientation.z} {initial_pose.orientation.w}\n")
#     rospy.loginfo("Initial pose recorded.")

# def main():
#     rospy.init_node('record_initial_pose', anonymous=True)
#     rospy.wait_for_message("/odom", Odometry, odom_callback)
    

# if __name__ == '__main__':
#     main()



import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped  # Use PoseWithCovarianceStamped for SLAM
import os

class InitialPoseRecorder:
    def __init__(self):
        self.initial_position = None
        self.file_path = os.path.expanduser("~/initial_pose.txt")
        rospy.init_node('record_initial_position')
        rospy.Subscriber('/slam_out_pose', PoseWithCovarianceStamped, self.pose_callback)
        rospy.loginfo("Subscriber to /slam_out_pose is set up.")
        
    def pose_callback(self, msg):
        if self.initial_position is None:
            self.initial_position = msg.pose.pose
            rospy.loginfo("Initial position recorded: %s", self.initial_position)
            self.save_pose_to_file()

    def save_pose_to_file(self):
        try:
            with open(self.file_path, "w") as file:
                pose = self.initial_position
                file.write(f"{pose.position.x} {pose.position.y} "
                           f"{pose.orientation.z} {pose.orientation.w}\n")
            rospy.loginfo("Initial pose saved to file.")
        except IOError as e:
            rospy.logerr("Failed to write initial pose to file: %s", str(e))

def main():
    recorder = InitialPoseRecorder()
    rospy.spin()

if __name__ == '__main__':
    main()
