#!/usr/bin/env python3
# import rospy
# from geometry_msgs.msg import PoseStamped, Point, Quaternion

# def publish_goal(x, y, yaw):
#     # Create a PoseStamped message
#     goal = PoseStamped()
    
#     # Set the frame ID and timestamp
#     goal.header.frame_id = "base_footprint"
#     goal.header.stamp = rospy.Time.now()
    
#     # Set the goal's position and orientation
#     goal.pose.position = Point(x, y, 0.0)
#     goal.pose.orientation = Quaternion(0.0, 0.0, yaw, 1.0)
    
#     # Publish the goal
#     goal_pub.publish(goal)

# if __name__ == '__main__':
#     try:
#         rospy.init_node('goal_publisher_node')
        
#         # Create a publisher to the 'move_base_simple/goal' topic
#         goal_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
        
#         # Allow time for connections to establish
#         rospy.sleep(2)
        
#         # Example coordinates: move the robot to (1.0, 2.0) with a 45-degree yaw
#         x = 0.0
#         y = 0.0
#         yaw = 1.0  # 45 degrees in radians
        
#         # Publish the goal
#         publish_goal(x, y, yaw)
        
#         rospy.loginfo("Goal published!")

#     except rospy.ROSInterruptException:
#         rospy.logerr("Goal publishing interrupted.")



import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, Quaternion

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
    goal.target_pose.pose.position = Point(x, y, 0.0)
    goal.target_pose.pose.orientation = Quaternion(0.0, 0.0, yaw, 1.0)

    # Send the goal to move_base
    rospy.loginfo(f"Sending goal: Position({x}, {y}), Orientation({yaw})")
    client.send_goal(goal)

    # Wait for the result
    client.wait_for_result()

    # Check if the goal was achieved
    if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Goal reached successfully!")
    else:
        rospy.logwarn("Failed to reach goal. The robot might be stuck or the goal is unreachable.")

if __name__ == '__main__':
    try:
        # Initialize the ROS node
        rospy.init_node('move_base_client_node')

        # Example goal: move the robot to (1.0, 2.0) with a 45-degree yaw (in radians)
        x = 0.0
        y = 0.0
        yaw = 1.0  # 45 degrees in radians

        # Call the function to send the goal
        movebase_client(x, y, yaw)

    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")

# import rospy
# from nav_msgs.msg import Odometry
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# import actionlib
# from geometry_msgs.msg import Point, Quaternion

# def odom_callback(msg):
#     """Callback function to handle incoming Odometry data."""
#     pose = msg.pose.pose
#     rospy.loginfo("Current Position - x: %f, y: %f, z: %f", pose.position.x, pose.position.y, pose.position.z)
#     rospy.loginfo("Current Orientation - x: %f, y: %f, z: %f, w: %f", pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)

# def move_to_goal(x, y, z, w):
#     """Send a goal position to move_base."""
#     client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
#     client.wait_for_server()

#     goal = MoveBaseGoal()
#     goal.target_pose.header.frame_id = "map"
#     goal.target_pose.header.stamp = rospy.Time.now()
#     goal.target_pose.pose.position = Point(x, y, 0.0)
#     goal.target_pose.pose.orientation = Quaternion(0.0, 0.0, z, w)

#     rospy.loginfo("Sending goal to position: x=%f, y=%f", x, y)
#     client.send_goal(goal)
#     client.wait_for_result()

#     if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
#         rospy.loginfo("Goal reached successfully")
#     else:
#         rospy.logerr("Failed to reach goal")

# def main():
#     rospy.init_node('navigation_example')
#     rospy.Subscriber('/odom', Odometry, odom_callback)
#     rospy.sleep(10)  # Allow time for subscription to start
#     move_to_goal(1.0, 2.0, 0.0, 1.0)

# if __name__ == '__main__':
#     main()

