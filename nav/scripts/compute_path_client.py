# In compute_path_client.py or another node

from geometry_msgs.msg import PoseStamped
from your_package.srv import Dijkstra
import rospy

start = None
goal = None

def start_callback(msg):
    global start
    start = msg.pose.position
    rospy.loginfo(f"Start set to: {start}")

def goal_callback(msg):
    global goal
    goal = msg.pose.position
    rospy.loginfo(f"Goal set to: {goal}")
    if start is not None:
        rospy.wait_for_service('compute_dijkstra_path')
        try:
            compute_path = rospy.ServiceProxy('compute_dijkstra_path', Dijkstra)
            response = compute_path(start, goal)
            rospy.loginfo("Path received")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

def listener():
    rospy.init_node('path_requester')
    rospy.Subscriber('/start_pose', PoseStamped, start_callback)
    rospy.Subscriber('/goal_pose', PoseStamped, goal_callback)
    rospy.spin()

