import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import json
import random
import operator
from cpt.cpt import Cpt
import numpy as np
import pickle

class MarkerToActionBridge(Node):
    def __init__(self):
        super().__init__('marker_to_action_bridge')
        
        self.subscription = self.create_subscription(String, '/detected_markers', self.marker_callback, 10)
        #self.query_subscription = self.create_subscription(Bool, '/query', self.query_callback, 10)
        self.publisher = self.create_publisher(String, '/command', 10)
        
        self.previous_sequence = []
        self.marker_map = {
            0: 'B1', 1: 'B2', 2: 'B3', 3: 'B4',
            4: 'L12', 5: 'L23', 6: 'L34', 7: 'L14',
            8: 'T1', 9: 'T2', 10: 'T3', 11: 'T4'
        }

        self.trees = self.get_trees()

    def get_trees(self):
        
        trees = pickle.load(open('trained_trees.pickle','rb'))

        self.get_logger().info(f"CPT trees loaded: {trees}")  # ROS 2 logging
        
        return trees

    
    def marker_callback(self, msg):
        #try:
            valid_json_string = msg.data.replace("''", '"').replace("'", '"')
            detected_markers = json.loads(valid_json_string)
            if "ids" in detected_markers:
                detected_ids = detected_markers["ids"]
                translated_markers = [self.marker_map[i] for i in detected_ids if i in self.marker_map]
                
                # Store unique markers while preserving order
                for marker in translated_markers:
                    if marker not in self.previous_sequence:
                        self.previous_sequence.append(marker)

                # FEED SEQUENCE TO TRAINED MODEL AND GET PREDICTION
                self.get_logger().info(f"Feeding sequence: {self.previous_sequence}")

                predicted_step = self.predict_next_step(self.previous_sequence)

                self.get_logger().info(f"Predicted next step: {predicted_step}")
                
                self.publisher.publish(String(data=predicted_step))                
                
                # Clear the sequence for the next query
                self.previous_sequence = []
        #except json.JSONDecodeError:
        #    self.get_logger().error("Failed to decode marker message")
    '''
    def query_callback(self, msg):
        if msg.data:  # Only process when Bool is True
            if self.previous_sequence:
                # FEED SEQUENCE TO TRAINED MODEL AND GET PREDICTION
                self.get_logger().info(f"Feeding sequence: {self.previous_sequence}")

                predicted_step = self.predict_next_step(self.previous_sequence)

                self.get_logger().info(f"Predicted next step: {predicted_step}")
                
                self.publisher.publish(String(data=predicted_step))                
                
                # Clear the sequence for the next query
                self.previous_sequence = []
    '''
    def predict_next_step(self, sequence):
        # Placeholder: Replace with actual model logic
        next_step = self.trees[int(len(sequence)+1)].predict_k([sequence], 15)[0][0]
        return next_step  # Modify this with real prediction logic

def main(args=None):
    rclpy.init(args=args)
    node = MarkerToActionBridge()
    rclpy.spin(node)
    rclpy.shutdown()
