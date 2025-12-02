#!/usr/bin/env python3
"""
Distance Estimator Node using Neural Network
This node processes camera images to estimate distances to objects
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray, Header, MultiArrayDimension
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
import torch.nn as nn
from typing import List, Tuple

class DistanceEstimationNet(nn.Module):
    """
    Simple CNN for distance estimation from camera images
    This is a placeholder that can be replaced with a pre-trained model
    """
    def __init__(self):
        super(DistanceEstimationNet, self).__init__()
        self.conv1 = nn.Conv2d(3, 32, kernel_size=3, padding=1)
        self.conv2 = nn.Conv2d(32, 64, kernel_size=3, padding=1)
        self.conv3 = nn.Conv2d(64, 128, kernel_size=3, padding=1)
        self.pool = nn.MaxPool2d(2, 2)
        self.fc1 = nn.Linear(128 * 28 * 28, 256)
        self.fc2 = nn.Linear(256, 128)
        self.fc3 = nn.Linear(128, 10)  # Output: 10 distance points
        self.relu = nn.ReLU()
        
    def forward(self, x):
        x = self.pool(self.relu(self.conv1(x)))
        x = self.pool(self.relu(self.conv2(x)))
        x = self.pool(self.relu(self.conv3(x)))
        x = x.view(-1, 128 * 28 * 28)
        x = self.relu(self.fc1(x))
        x = self.relu(self.fc2(x))
        x = self.fc3(x)
        return x

class DistanceEstimatorNode(Node):
    """
    ROS 2 Node for estimating distances to objects using neural network
    """
    
    def __init__(self):
        super().__init__('distance_estimator_node')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Initialize neural network
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.get_logger().info(f'Using device: {self.device}')
        
        self.model = DistanceEstimationNet().to(self.device)
        self.model.eval()
        
        # For now, use a simple depth estimation approach
        # In production, load a pre-trained model here
        # self.model.load_state_dict(torch.load('path_to_model.pth'))
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publishers
        self.distance_pub = self.create_publisher(
            Float32MultiArray,
            '/robot/object_distances',
            10
        )
        
        self.processed_image_pub = self.create_publisher(
            Image,
            '/robot/processed_image',
            10
        )
        
        self.get_logger().info('Distance Estimator Node initialized')
        
    def estimate_depth_simple(self, cv_image: np.ndarray) -> np.ndarray:
        """
        Simple depth estimation using image processing
        In production, this would use the neural network
        """
        # Convert to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Simple depth estimation based on edge density and focus
        # This is a placeholder - replace with actual neural network inference
        edges = cv2.Canny(blurred, 50, 150)
        edge_density = np.sum(edges) / (edges.shape[0] * edges.shape[1])
        
        # Create a simple depth map (farther objects = darker)
        depth_map = cv2.GaussianBlur(gray, (15, 15), 0)
        depth_map = 255 - depth_map  # Invert for visualization
        
        # Estimate distances at specific points (grid sampling)
        h, w = cv_image.shape[:2]
        num_rows = 10
        num_cols = 5
        distances = []
        
        for i in range(num_rows):
            y = int(h * (i + 1) / (num_rows + 1))
            for j in range(num_cols):
                x = int(w * (j + 1) / (num_cols + 1))
                # Simple distance estimation (in meters)
                # Higher intensity = closer object
                intensity = gray[y, x]
                # Approximate distance: 0.5m to 10m range
                distance = 10.0 - (intensity / 255.0) * 9.5
                distances.append(distance)
        
        return np.array(distances)
    
    def image_callback(self, msg: Image):
        """
        Callback function for processing camera images
        """
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Estimate distances using neural network (or simple method)
            distances = self.estimate_depth_simple(cv_image)
            
            # Publish distance array
            distance_msg = Float32MultiArray()
            distance_msg.data = distances.tolist()
            # Set layout dimensions (optional, but helpful for debugging)
            dim = MultiArrayDimension()
            dim.label = "distances"
            dim.size = len(distances)
            dim.stride = len(distances)
            distance_msg.layout.dim.append(dim)
            self.distance_pub.publish(distance_msg)
            
            # Draw distance information on image for visualization
            h, w = cv_image.shape[:2]
            num_rows = 10
            num_cols = 5
            idx = 0
            for i in range(num_rows):
                y = int(h * (i + 1) / (num_rows + 1))
                for j in range(num_cols):
                    x = int(w * (j + 1) / (num_cols + 1))
                    if idx < len(distances):
                        distance = distances[idx]
                        cv2.circle(cv_image, (x, y), 5, (0, 255, 0), -1)
                        cv2.putText(cv_image, f'{distance:.1f}m', 
                                  (x+10, y), cv2.FONT_HERSHEY_SIMPLEX, 
                                  0.3, (0, 255, 0), 1)
                        idx += 1
            
            # Publish processed image
            processed_image_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
            processed_image_msg.header = msg.header
            self.processed_image_pub.publish(processed_image_msg)
            
            self.get_logger().debug(f'Estimated {len(distances)} distance points')
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = DistanceEstimatorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

