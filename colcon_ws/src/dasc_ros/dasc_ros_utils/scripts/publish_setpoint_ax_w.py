#!/usr/bin/env python3
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import TrajectorySetpoint, VehicleLocalPosition, VehicleAngularVelocity, VehicleAcceleration
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
        self.publisher = self.create_publisher(VehicleAngularVelocity, '/px4_1/fmu/in/vehicle_angular_velocity', 10)
        self.publisher2 = self.create_publisher(VehicleAcceleration, '/px4_1/fmu/in/vehicle_acceleration', 10)
        # self.timer = self.create_timer(0.1, self.publish_text)
        self.subscription = self.create_subscription(VehicleLocalPosition,'/px4_1/fmu/out/vehicle_local_position',self.listener_callback,QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        self.msg = VehicleAngularVelocity()
        self.msg2 = VehicleAcceleration()
        self.v=0.56
        self.start_time=time.time()

        while True:
            self.current_time=time.time()
            diff=self.current_time-self.start_time
            self.msg.xyz = [-self.v, 0.0, 0.0]
            # self.publisher.publish(self.msg)
            self.msg2.xyz = [self.v,self.v,0.0]
            self.publisher2.publish(self.msg2)
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

