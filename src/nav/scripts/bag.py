#!/usr/bin/env python3
from ultralytics import YOLO
import pyrealsense2 as rs
import numpy as np
import cv2
import rospy
import tf2_ros
import tf2_geometry_msgs
import mediapipe as mp
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Point, Quaternion
from tf.transformations import quaternion_from_euler

# Initialize ROS node
rospy.init_node('bag_detection_node')

# Define publishers
object_class_pub = rospy.Publisher('/detected_object_class', String, queue_size=10)
object_distance_pub = rospy.Publisher('/detected_object_distance', Float32, queue_size=10)
object_coords_pub = rospy.Publisher('/detected_object_coords', Point, queue_size=10)
object_orien_pub = rospy.Publisher('/detected_object_orientation', Quaternion, queue_size=10)

# TF2 buffer and listener for transforming coordinates
tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

# Load YOLOv8 models
person_model = YOLO('yolov8m.pt')  # Pretrained YOLOv8 model for detecting persons
bag_model = YOLO('/home/upm-m15/Downloads/last.pt')  # Replace with your trained model path

# Initialize MediaPipe Pose and Hands modules
mp_pose = mp.solutions.pose
pose = mp_pose.Pose()
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(static_image_mode=False, max_num_hands=1, min_detection_confidence=0.65)

# Initialize RealSense camera
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)  # Reduce frame rate to 15 FPS
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
pipeline.start(config)

# Camera intrinsics
intrinsics = {
    'width': 640,
    'height': 480,
    'ppx': 323.8177795410156,  # Principal point x
    'ppy': 237.62747192382812,  # Principal point y
    'fx': 610.447509765625,  # Focal length x
    'fy': 608.9990844726562,  # Focal length y
    'distortion_model': 'distortion.inverse_brown_conrady',
    'coeffs': [0.0, 0.0, 0.0, 0.0, 0.0]  # Distortion coefficients
}

def convert_depth_to_phys_coord_using_realsense(x, y, depth, intrinsics):
    """Convert depth and pixel coordinates to physical coordinates."""
    cx = intrinsics['ppx']
    cy = intrinsics['ppy']
    fx = intrinsics['fx']
    fy = intrinsics['fy']

    X = (x - cx) * depth / fx
    Y = (y - cy) * depth / fy
    Z = depth
    return X, Y, Z

def transform_to_map_frame(x, y, z, listener):
    """Transform coordinates from camera frame to map frame."""
    try:
        # Create a point in the camera frame
        point_camera = PointStamped()
        point_camera.header.frame_id = "camera_depth_frame"
        point_camera.header.stamp = rospy.Time.now()
        point_camera.point.x = x
        point_camera.point.y = y
        point_camera.point.z = z

        # Transform the point to the map frame
        point_map = listener.transformPoint("map", point_camera)

        # Return the transformed coordinates
        return point_map.point.x, point_map.point.y, point_map.point.z
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logerr("TF Exception: %s", e)
        return None, None, None

def calculate_hand_pointing_vector(landmarks, depth_image):
    """Calculate the hand pointing vector."""
    wrist = landmarks[mp_hands.HandLandmark.WRIST]
    index_finger = landmarks[mp_hands.HandLandmark.INDEX_FINGER_TIP]

    # Convert normalized landmarks to pixel coordinates
    wrist_x = int(wrist.x * depth_image.shape[1])
    wrist_y = int(wrist.y * depth_image.shape[0])
    index_x = int(index_finger.x * depth_image.shape[1])
    index_y = int(index_finger.y * depth_image.shape[0])

    # Get depth values at wrist and index finger positions
    depth_wrist = depth_image[wrist_y, wrist_x]
    depth_index = depth_image[index_y, index_x]

    # Convert to real-world coordinates
    wrist_coords = convert_depth_to_phys_coord_using_realsense(wrist_x, wrist_y, depth_wrist, intrinsics)
    index_coords = convert_depth_to_phys_coord_using_realsense(index_x, index_y, depth_index, intrinsics)

    # Calculate pointing vector (from wrist to index finger tip)
    pointing_vector = np.array(index_coords) - np.array(wrist_coords)

    # For the purpose of direct publishing, return a dummy quaternion
    # You need to replace this with the actual quaternion if available
    quaternion = [0.0, 0.0, 0.0, 1.0]  # Placeholder quaternion (XYZW)

    return wrist_coords, pointing_vector, quaternion

def find_pointed_object(wrist_coords, pointing_vector, detected_objects):
    """Find the object being pointed at based on the pointing vector."""
    closest_object_coords = None
    closest_object_class = None
    min_distance = float('inf')

    for obj in detected_objects:
        obj_x1, obj_y1, obj_x2, obj_y2, obj_depth, obj_class = obj
        obj_coords = convert_depth_to_phys_coord_using_realsense((obj_x1 + obj_x2) // 2, (obj_y1 + obj_y2) // 2, obj_depth, intrinsics)

        # Check if pointing vector intersects with object's bounding box
        distance = np.linalg.norm(np.cross(pointing_vector, np.array(obj_coords) - np.array(wrist_coords)))
        
        # Check the shortest distance
        if distance < 0.2:  # Adjust threshold as needed
            return obj_coords, obj_class

    return None, None

def publish_coordinates(x, y, z, orientation):
    """Publish the detected object coordinates and orientation in the map frame."""
    target_point = Point()
    target_point.x = x
    target_point.y = y
    target_point.z = z
    object_coords_pub.publish(target_point)

    target_orientation = Quaternion()
    target_orientation.x = orientation[0]
    target_orientation.y = orientation[1]
    target_orientation.z = orientation[2]
    target_orientation.w = orientation[3]
    object_orien_pub.publish(target_orientation)

    rospy.loginfo(f"Published coordinates in map frame: ({x:.2f}, {y:.2f}, {z:.2f}) with orientation: {orientation}")

def detect_and_publish_bag_coordinates():
    """Detect bags and publish coordinates in the map frame."""
    bag_detected = False  # Flag to check if bag is detected

    try:
        while not rospy.is_shutdown():  # ROS main loop
            try:
                # Get frame data with a longer timeout
                frames = pipeline.wait_for_frames(timeout_ms=10000)
            except RuntimeError as e:
                rospy.logerr(f"Failed to get frames: {e}")
                continue

            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()

            if not depth_frame or not color_frame:
                continue

            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # Use YOLOv8 for person detection
            person_results = person_model(color_image, conf=0.65)
            detected_objects = []

            # Process person detection results
            for result in person_results:
                if result.boxes is not None:
                    for box in result.boxes:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        class_id = box.cls[0].item()
                        label = result.names[class_id]

                        if class_id == 0:  # Class ID 0 corresponds to "person"
                            depth = depth_image[(y1 + y2) // 2, (x1 + x2) // 2] * depth_frame.get_units()
                            detected_objects.append((x1, y1, x2, y2, depth, class_id))

                            # Convert to physical coordinates
                            X, Y, Z = convert_depth_to_phys_coord_using_realsense((x1 + x2) // 2, (y1 + y2) // 2, depth, intrinsics)
                            rospy.loginfo(f"Person detected at (X, Y, Z): ({X:.2f}, {Y:.2f}, {Z:.2f})")

            # Process bag detection
            bag_results = bag_model(color_image, conf=0.65)
            for result in bag_results:
                if result.boxes is not None:
                    for box in result.boxes:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        class_id = box.cls[0].item()
                        label = result.names[class_id]

                        depth = depth_image[(y1 + y2) // 2, (x1 + x2) // 2] * depth_frame.get_units()
                        detected_objects.append((x1, y1, x2, y2, depth, class_id))

                        # Convert to physical coordinates
                        X, Y, Z = convert_depth_to_phys_coord_using_realsense((x1 + x2) // 2, (y1 + y2) // 2, depth, intrinsics)
                        rospy.loginfo(f"Bag detected at (X, Y, Z): ({X:.2f}, {Y:.2f}, {Z:.2f})")

            # Process hand detection
            image_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
            results = hands.process(image_rgb)
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    wrist_coords, pointing_vector, orientation = calculate_hand_pointing_vector(hand_landmarks.landmark, depth_image)
                    closest_object_coords, closest_object_class = find_pointed_object(wrist_coords, pointing_vector, detected_objects)
                    if closest_object_coords:
                        # Transform coordinates to map frame
                        x_map, y_map, z_map = transform_to_map_frame(*closest_object_coords, tf_listener)
                        if x_map is not None:
                            # Publish coordinates and orientation
                            publish_coordinates(x_map, y_map, z_map, orientation)
                            bag_detected = True
                            rospy.loginfo(f"Map frame coordinate of selected bag: ({x_map:.2f}, {y_map:.2f}, {z_map:.2f}) with orientation: {orientation}")
                            break

            if bag_detected:
                break  # Exit after publishing

            # Display the resulting frame
            cv2.imshow('RealSense', color_image)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        # Stop the RealSense pipeline
        pipeline.stop()

        # Close all OpenCV windows
        cv2.destroyAllWindows()

if __name__ == '__main__':
    detect_and_publish_bag_coordinates()
