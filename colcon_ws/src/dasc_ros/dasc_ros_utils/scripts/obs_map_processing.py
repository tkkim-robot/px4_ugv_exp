#!/usr/bin/env python3
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.node import Node
from nvblox_msgs.msg import DistanceMapSlice
import numpy as np
import time
import cv2
# from sklearn.cluster import DBSCAN
from math import ceil
from std_msgs.msg import String

class ObsMapProcNode(Node):
    def __init__(self):
        super().__init__('obs_map_processor')
        #self.publisher = self.create_publisher(TrajectorySetpoint, '/px4_1/fmu/in/trajectory_setpoint', 10)
        self.subscription = self.create_subscription(DistanceMapSlice,'/nvblox_node/static_map_slice',self.nvblox_map_callback,QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.publisher = self.create_publisher(String,'/obstacle_circles',10)  # Added publisher initialization for String
        self.nan_number = 1000.0
        self.epsilon=10
    
    # Function to find clusters of white pixels and their minimum enclosing circles
    def find_obstacle_clusters(self,image):
        # Convert to grayscale and threshold to find white pixels
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray, 254, 255, cv2.THRESH_BINARY)
        
        # Find contours of the white regions
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Initialize a list to store circle information
        circles = []
        
        # Create a copy of the image for drawing
        output_image = image.copy()
        
        # Iterate through each contour and find minimum enclosing circle
        for i, contour in enumerate(contours):
            # Find minimum enclosing circle
            (x, y), radius = cv2.minEnclosingCircle(contour)
            center = (int(x), int(y))
            radius = int(ceil(radius))
            
            # Append to circles list
            circles.append((center, radius))
            
            # Draw the circle on the image
            cv2.circle(output_image, center, radius, (0, 0, 255), 2)
            
            # Print x, y, radius of each circle
            print(f"Circle {i + 1}: Center = {center}, Radius = {radius}")
        
        return output_image, circles

    def nvblox_map_callback(self, msg):

        static_map_slice = msg.data
        width = msg.width
        height = msg.height
        resolution = msg.resolution

        static_map_slice = np.array(static_map_slice).reshape((height, width))
        
        # find self.nan_number and store the indices of them as mask
        mask = np.isnan(static_map_slice)
        static_map_slice *= 100
        static_map_slice = static_map_slice.astype(np.uint8)
        cv2.imwrite("/workspaces/colcon_ws/static_map_slice1.png", static_map_slice)
        # find values close to zero with small epsilon, and mark them as highest value (like 255)
        static_map_slice[static_map_slice < self.epsilon] = 255
        # Taking a matrix of size 5 as the kernel 
        kernel = np.ones((5, 5), np.uint8) 
        # TODO: do erode dilate to remove small noise
        img_erosion = cv2.erode(static_map_slice, kernel, iterations=1) 
        static_map_slice = cv2.dilate(img_erosion, kernel, iterations=1) 
        # TODO: apply some sort of clustering method to cluster the close obs indices into groups
        # TODO: find a minimum circle that covers the cluster
        # TODO: publish set of circles as [x,y,radius] to ROS2 

        static_map_slice = cv2.cvtColor(static_map_slice, cv2.COLOR_BGR2RGB)
        obs_indices = np.where(static_map_slice < 1)

        cv2.imwrite("/workspaces/colcon_ws/static_map_slice2.png", static_map_slice)

        print(np.shape(static_map_slice))

        # Find obstacle clusters and draw circles
        image_with_circles, circles = self.find_obstacle_clusters(static_map_slice)
        print(circles)

        # Prepare circles information as a flat list of integers (x, y, radius)
        circles_data = []
        for circle in circles:
            circles_data.extend(circle)
        obs_info_str=str(circles_data)
        print("Data string: ",obs_info_str)

        # Show the image with circles
        cv2.imwrite("/workspaces/colcon_ws/obstacle_slice.png", image_with_circles)
        # Publish circles information as String
        circles_msg = String()
        circles_msg.data = obs_info_str
        self.publisher.publish(circles_msg)

        # cv2.imshow("asdf", static_map_slice)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    obs_map_processing_node = ObsMapProcNode()
    rclpy.spin(obs_map_processing_node)
    obs_map_processing_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

