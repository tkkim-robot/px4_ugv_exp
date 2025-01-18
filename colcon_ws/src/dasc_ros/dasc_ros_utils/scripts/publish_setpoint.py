#!/usr/bin/env python3
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import TrajectorySetpoint, VehicleLocalPosition
import numpy as np
from scipy.interpolate import CubicSpline
import time
# import keyboard
# from pynput import keyboard
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
        self.vel_x = 0.0 # Linear acceleration in m/s^2
        self.yaw_rate = 0 # Yaw rate in rad/sec ; +ve moves left, -ve moves right
        self.start_time = time.time()
        self.L = 0.3302 # Wheel base in m

        while True:
            key = input("Enter command (w/a/s/d to move, any other key to stop): ").strip().lower()
            if key=='s':
                print("back")
                self.vel_x = 0.5
                self.yaw_rate = 0
            elif key=='a':
                print("left")
                self.vel_x = 0
                self.yaw_rate = 3.0
            elif key=='w':
                print("front")
                self.vel_x = -0.5
                self.yaw_rate = 0
            elif key=='d':
                print("right")
                self.vel_x = 0
                self.yaw_rate = -3.0
            else:
                print("stop")
                self.vel_x = 0
                self.yaw_rate = 0
            self.current_time=time.time()
            diff=self.current_time-self.start_time
            self.msg.raw_mode=True
            v_l = 0.5 * (self.vel_x - 0.5 * self.L * self.yaw_rate)
            v_r = 0.5 * (self.vel_x + 0.5 * self.L * self.yaw_rate)
            print("Left vel is ", -v_l," and right vel is ", v_r)
            self.msg.cmd = [-v_l, v_r, 0.0,0.0]
            self.publisher.publish(self.msg)
            self.get_logger().info('Publishing setpoint')
            print(diff)
        


    def listener_callback(self, msg):
        global received_x,received_y
        # print("Entered subscriber")
        # self.get_logger().info('I heard: "%s"' % msg)
        # self.get_logger().info('%f, %f' % (msg.x, msg.y))
        received_x.append(msg.x)
        received_y.append(msg.y)
        print("X and Y is %f, %f" %(msg.x,msg.y))



def main(args=None):
    rclpy.init(args=args)
    setpoint_assigner_node = SetpointAssignerNode()
    rclpy.spin(setpoint_assigner_node)
    setpoint_assigner_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

