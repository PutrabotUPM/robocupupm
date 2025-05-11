import rospy
from geometry_msgs.msg import PoseStamped
from your_package.srv import Dijkstra
from geometry_msgs.msg import Point

# Global variables to store the received positions
start_position = None
goal_position = None

def start_pose_callback(msg):
    """
    Callback function for receiving the start position.
    """
    global start_position
    start_position = msg.pose.position
    rospy.loginfo(f"Start position updated: {start_position}")

def goal_pose_callback(msg):
    """
    Callback function for receiving the goal position.
    """
    global goal_position
    goal_position = msg.pose.position
    rospy.loginfo(f"Goal position updated: {goal_position}")

    if start_position is not None:
        rospy.loginfo("Both start and goal positions received. Requesting path.")
        path = request_path(start_position, goal_position)  # Use the request path function from earlier
        rospy.loginfo(f"Calculated path: {path}")

def request_path(start, goal):
    """
    Request a path from the Dijkstra service.
    """
    rospy.wait_for_service('compute_dijkstra_path')
    try:
        compute_path_service = rospy.ServiceProxy('compute_dijkstra_path', Dijkstra)
        response = compute_path_service(start, goal)
        rospy.loginfo("Path calculated successfully.")
        return response.path
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def listener():
    """
    Initialize the ROS node and subscribe to topics.
    """
    rospy.init_node('path_listener_node')

    # Subscribe to the start and goal positions
    rospy.Subscriber('/start_pose', PoseStamped, start_pose_callback)
    rospy.Subscriber('/goal_pose', PoseStamped, goal_pose_callback)

    rospy.spin()

if __name__ == '__main__':
    listener()

