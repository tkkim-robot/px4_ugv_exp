#!/usr/bin/env python3
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.node import Node
from nvblox_msgs.msg import DistanceMapSlice
import numpy as np
import time
import cv2


class ObsMapProcNode(Node):
    def __init__(self):
        super().__init__('obs_map_processor')
        #self.publisher = self.create_publisher(TrajectorySetpoint, '/px4_1/fmu/in/trajectory_setpoint', 10)
        self.subscription = self.create_subscription(DistanceMapSlice,'/nvblox_node/static_map_slice',self.nvblox_map_callback,QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.start_time=time.time()

        self.nan_number = 1000.0

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
        static_map_slice[static_map_slice < 1] = 255
        # TODO: define self.epsilon in init function
        # TODO: do erode dilate to remove small noise
        # TODO: apply some sort of clustering method to cluster the close obs indices into groups
        # TOOD: find a minimum circle that covers the cluster
        # TODO: publish set of circles as [x,y,radius] to ROS2
        obs_indices = np.where(static_map_slice < self.epsilon)
        # 

        static_map_slice = cv2.cvtColor(static_map_slice, cv2.COLOR_BGR2RGB)

        cv2.imwrite("/workspaces/colcon_ws/static_map_slice2.png", static_map_slice)


        cv2.imshow("asdf", static_map_slice)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    obs_map_processing_node = ObsMapProcNode()
    rclpy.spin(obs_map_processing_node)
    obs_map_processing_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

