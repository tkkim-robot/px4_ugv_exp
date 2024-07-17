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
        self.subscription = self.create_subscription(DistanceMapSlice,'/nvblox_node/map_slice',self.nvblox_map_callback,QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.publisher = self.create_publisher(String,'/obstacle_circles',10)  # Added publisher initialization for String
        # self.nan_number = 1000.0
        self.epsilon=0.2
    
    # Function to find clusters of white pixels and their minimum enclosing circles
    def find_obstacle_clusters(self, image, resolution, max_rad=1):
        # Convert to grayscale and threshold to find white pixels
        _, mask = cv2.threshold(image, 254, 255, cv2.THRESH_BINARY)
        # Find contours of the white regions
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # Initialize a list to store circle information
        obs_circles = []
        # Iterate through each contour and find minimum enclosing circle
        for i, contour in enumerate(contours):
            # Find minimum enclosing circle
            (x, y), radius = cv2.minEnclosingCircle(contour)
            radius = int(ceil(radius))
            if radius*resolution > max_rad:
                continue
            # Append to obs_circles list
            obs_circles.append((int(x),int(y), radius))
        return np.array(obs_circles, dtype=float)

    def cvt_pixel_to_global(self,obs_px,obs_py,map_origin_px,map_origin_py,resolution):
        delta_px=obs_px-map_origin_px
        # print("del ",type(delta_px))
        delta_py=obs_py-map_origin_py
        delta_x=delta_px*resolution
        delta_y=-1*delta_py*resolution
        global_x,global_y=delta_x,delta_y # since origin is always (0,0)
        # print("global ",type(global_x))
        # print(global_x,global_y)
        return float(global_x),float(global_y)


    def nvblox_map_callback(self, msg):
        map_slice = msg.data
        width = msg.width
        height = msg.height
        resolution = msg.resolution
        unknown_value =msg.unknown_value

        origin_px = msg.origin.x
        origin_py = msg.origin.y

        map_slice = np.array(map_slice).reshape((height, width)) # 2D
        print("Origin: ",origin_px," ,",origin_py)
        map_slice_vis = cv2.cvtColor( map_slice.copy(), cv2.COLOR_GRAY2BGR)

        # known values
        mask = np.where(map_slice!=unknown_value)

        # find values close to zero with small epsilon, and mark them as highest value (like 255)
        obs_map = np.zeros_like(map_slice)
        obs_map[map_slice < self.epsilon] = 255 # 1 is obstacle contour
        obs_map = obs_map.astype(np.uint8)

        # remove noise
        kernel = np.ones((8, 8), np.uint8) 
        obs_map = cv2.erode(obs_map, kernel, iterations=1) 
        obs_map = cv2.dilate(obs_map, kernel, iterations=1) 

        # Find obstacle clusters
        obs_circles = self.find_obstacle_clusters(obs_map, resolution)

        obs_global_frame = []
        for circle in obs_circles:
            px = circle[0]
            py = circle[1]
            radius = circle[2]
            # Draw the circle on the image
            cv2.circle(map_slice_vis, (int(px),int(py)), radius, (0, 0, 255), 2)
            # Print px, py, radius of each circle
            print(f"Circle {i + 1}: Center = {int(px), int(py)}, Radius = {radius}")

            # convert origin in pixel to global (fix axis)
            map_origin_px = -origin_px / resolution
            map_origin_py = -origin_py / resolution

            # convert to meter scale
            global_x, global_y = self.cvt_pixel_to_global(px, py, map_origin_px, map_origin_py, resolution)
            radius *= resolution
            obs_global_frame.append([global_x, global_y, radius])


        
        # Removing bigger circles
        rad_col = circles[:, 2]
        mask = rad_col <= 1
        circles = circles[mask]
        #Sorting by 2-norm
        sqrt_sum_squares = np.sqrt(circles[:, 0]**2 + circles[:, 1]**2)
        sorted_indices = np.argsort(sqrt_sum_squares)
        circles = circles[sorted_indices]
        obs_info_str=str(circles)
        else:
            obs_info_str=""
        print("Data string: ",obs_info_str)

        # Show the image with circles
        cv2.imwrite("/workspaces/colcon_ws/obstacle_slice.png", image_with_circles)
        # Publish circles information as String
        circles_msg = String()
        circles_msg.data = obs_info_str
        self.publisher.publish(circles_msg)


def main(args=None):
    rclpy.init(args=args)
    obs_map_processing_node = ObsMapProcNode()
    rclpy.spin(obs_map_processing_node)
    obs_map_processing_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

