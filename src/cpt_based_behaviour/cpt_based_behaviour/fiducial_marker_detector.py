import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from collections import deque, defaultdict

class FiducialMarkerDetector(Node):
    def __init__(self):
        super().__init__('fiducial_marker_detector')
        
        # Subscribe to the camera image feed
        self.subscription = self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        
        # Subscribe to the /query Bool topic
        self.query_subscription = self.create_subscription(Bool, '/query', self.query_callback, 10)
        
        # Publisher for detected markers
        self.publisher = self.create_publisher(String, '/detected_markers', 10)
        
        # Bridge to convert ROS Image to OpenCV format
        self.bridge = CvBridge()
        
        # Set up ArUco dictionary and parameters for marker detection
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.aruco_params = cv2.aruco.DetectorParameters()
        
        # Queue to store the last 15 detected IDs
        self.id_history = deque(maxlen=7)
        
        # Store the detected marker IDs
        self.detected_ids = []

        # Occurrence count dictionary: tracks how many frames a marker has been detected
        self.id_count = defaultdict(int)

    def query_callback(self, msg):
        """Callback for the /query Bool topic."""
        if msg.data:  # Only process if the received Bool value is True
            '''
            self.get_logger().info("Received True from /query. Processing marker detection...")

            # If there are no detected markers, return early
            if not self.detected_ids:
                self.get_logger().info("No marker detected yet.")
                return

            # Check if the detected IDs have been consistent for 15 frames
            if len(self.id_history) == 7 and all(ids == self.id_history[0] for ids in self.id_history):
                # If the IDs have remained consistent for 15 frames, publish the data
                self.get_logger().info(f"Detected Marker IDs consistently for 7 frames: {self.detected_ids}")
                
                # Publish marker data
                marker_data = {"ids": self.detected_ids}
                self.publisher.publish(String(data=str(marker_data)))
            else:
                self.get_logger().info("Detected markers are inconsistent over the last 15 frames or no consistent detection.")
            '''
            # If query was triggered, process and publish the results
        if len(self.id_history) == 7 and all(ids == self.id_history[0] for ids in self.id_history):
            self.get_logger().info("Query triggered. Processing detected markers.")
            
            # Filter IDs based on their occurrence across the 15 frames
            filtered_ids = self.filter_ids_by_occurrence()
            if filtered_ids:
                marker_data = {"ids": filtered_ids}
                self.get_logger().info("No markers with sufficient consecutive occurrences.")
                self.publisher.publish(String(data=str(marker_data)))
            else:
                self.get_logger().info("No markers with sufficient consecutive occurrences.")
        else:
            self.get_logger().info("Received False from /query. Not processing marker detection.")

    def image_callback(self, msg):
        """Callback for the /camera/camera/color/image_raw topic."""
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            # Convert ids to a list of integers
            detected_ids = [int(i) for i in ids.flatten() if 0 <= int(i) <= 11]
            self.get_logger().info(f"Feeding sequence: {set(detected_ids)}")

            # Add the detected IDs to the history
            self.id_history.append(set(detected_ids))

            # Reset counts for IDs that were not detected
            detected_ids_set = set(detected_ids)
            for id_ in list(self.id_count.keys()):
                if id_ not in detected_ids_set:
                    self.id_count[id_] = 0  # Reset the count for IDs that were not detected

            # Update the count for detected IDs
            for id_ in detected_ids:
                self.id_count[id_] += 1

            # Store the latest detected IDs to be used later when query is triggered
            self.detected_ids = detected_ids
            '''
            # If no new ids are detected, we append an empty list to history
            if not detected_ids:
                detected_ids = []

            # Convert to set to allow order-agnostic comparison
            detected_ids_set = set(detected_ids)
            
            self.get_logger().info(f"Detected Marker IDs: {detected_ids}")  # ROS 2 logging

            # Append detected ids to history
            self.id_history.append(detected_ids_set)

            # Store the latest detected IDs to be used when processing the query
            self.detected_ids = detected_ids
            '''
        else:
            #self.get_logger().info("No marker detected")
            self.id_history.clear()  # Clear the history as no marker was detected
            self.detected_ids = []  # Reset the detected_ids list
            self.id_count.clear()  # Reset all counts if no markers were detected

        

    def filter_ids_by_occurrence(self):
        """Filters marker IDs by their occurrence over the last 15 frames.
        Only returns IDs that appeared at least 7 times consecutively."""
        
        # Get the IDs that have been detected at least 7 times
        filtered_ids = [id_ for id_, count in self.id_count.items() if count >= 7]
        
        return filtered_ids


def main(args=None):
    rclpy.init(args=args)
    node = FiducialMarkerDetector()
    rclpy.spin(node)
    rclpy.shutdown()

'''import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
for id_ in list(self.id_count.keys()):
                if id_ not in detected_ids_set:
                    self.id_count[id_] = 0  # Reset the count for IDs that were not detected

            # Update the count for detected IDs
            for id_ in detected_ids:
                self.id_count[id_] += 1

            # Store the latest detected IDs to be used later when query is triggered
            self.detected_ids = detected_ids
class FiducialMarkerDetector(Node):
    def __init__(self):
        super().__init__('fiducial_marker_detector')
        self.subscription = self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        self.publisher = self.create_publisher(String, '/detected_markers', 10)
        self.bridge = CvBridge()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)

        
        if ids is not None:
            detected_ids = [int(i) for i in ids.flatten() if 0 <= int(i) <= 11]
            self.get_logger().info(f"Detected Marker IDs: {detected_ids}")  # ROS 2 logging
            marker_data = {"ids": ids.flatten().tolist()}
            self.publisher.publish(String(data=str(marker_data)))
        else:
            self.get_logger().info(f"No marker detected")  # ROS 2 logging


def main(args=None):
    rclpy.init(args=args)
    node = FiducialMarkerDefor id_ in list(self.id_count.keys()):
                if id_ not in detected_ids_set:
                    self.id_count[id_] = 0  # Reset the count for IDs that were not detected

            # Update the count for detected IDs
            for id_ in detected_ids:
                self.id_count[id_] += 1

            # Store the latest detected IDs to be used later when query is triggered
            self.detected_ids = detected_idstector()
    rclpy.spin(node)
    rclpy.shutdown()
'''