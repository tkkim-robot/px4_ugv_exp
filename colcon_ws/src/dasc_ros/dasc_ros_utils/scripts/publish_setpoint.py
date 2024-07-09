#!/usr/bin/env python3
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import TrajectorySetpoint, VehicleLocalPosition
import numpy as np
from scipy.interpolate import CubicSpline
import time
# import matplotlib
# matplotlib.use('GTK')  # Or any other X11 back-end
from geometry_msgs.msg import Point

received_x=[]
received_y=[]




class SetpointAssignerNode(Node):
    def __init__(self):
        super().__init__('setpoint_assigner')
        self.publisher = self.create_publisher(TrajectorySetpoint, '/px4_1/fmu/in/trajectory_setpoint', 10)
        # self.timer = self.create_timer(0.1, self.publish_text)
        self.subscription = self.create_subscription(VehicleLocalPosition,'/px4_1/fmu/out/vehicle_local_position',self.listener_callback,QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        self.msg = TrajectorySetpoint()
        self.acc_x=0.56 # Linear acceleration in m/s^2
        self.yaw_rate= 0 # Yaw rate in rad/sec
        self.start_time=time.time()
        self.L=0.3302 # Wheel base in m

        while True:
            self.current_time=time.time()
            diff=self.current_time-self.start_time
            self.msg.raw_mode=True
            v_l = 0.5 * (self.acc_x - 0.5 * self.L * self.yaw_rate)
            v_r = 0.5 * (self.acc_x + 0.5 * self.L * self.yaw_rate)
            print("Left vel is ", -v_l," and right vel is ", v_r)
            self.msg.cmd = [v_r, -v_l, 0.0,0.0]
            self.publisher.publish(self.msg)
            self.get_logger().info('Publishing setpoint')
            print(diff)

    # def publish_text(self):
    #     self.current_time=time.time()
    #     diff=self.current_time-self.start_time
    #     self.msg.xyz = [-self.v, 0.0, 0.0]
    #     # self.publisher.publish(self.msg)
    #     self.msg2.xyz = [-self.v,0.0,0.0]
    #     self.publisher2.publish(self.msg2)
    #     self.get_logger().info('Publishing setpoint')
    #     print(diff)
        


    def listener_callback(self, msg):
        global received_x,received_y
        # print("Entered subscriber")
        # self.get_logger().info('I heard: "%s"' % msg)
        # self.get_logger().info('%f, %f' % (msg.x, msg.y))
        received_x.append(msg.x)
        received_y.append(msg.y)
        # print("%f, %f" %(msg.x,msg.y))



def main(args=None):
    rclpy.init(args=args)
    setpoint_assigner_node = SetpointAssignerNode()
    rclpy.spin(setpoint_assigner_node)
    setpoint_assigner_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

