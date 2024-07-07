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
import matplotlib.pyplot as plt
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

received_x=[]
received_y=[]

class SetpointAssignerNode(Node):
    def __init__(self):
        super().__init__('setpoint_assigner')
        self.publisher = self.create_publisher(TrajectorySetpoint, '/px4_1/fmu/in/trajectory_setpoint', 10)
        # self.marker_pub=self.create_publisher(Marker,'path_marker',10)
        # qos_profile = QoSProfile(reliability=QoSProfile.ReliabilityPolicy.RELIABLE)
        self.timer = self.create_timer(0.1, self.publish_text)
        # self.subscription = self.create_subscription(VehicleLocalPosition,'/px4_1/fmu/out/vehicle_local_position',self.listener_callback,10)
        self.subscription = self.create_subscription(VehicleLocalPosition,'/px4_1/fmu/out/vehicle_local_position',self.listener_callback,QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        # self.subscription
        
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        

        self.msg = TrajectorySetpoint()
        self.num_points=400
        self.angles=np.linspace(0,2*np.pi,self.num_points)
        # # Shape 8
        self.x=np.cos(self.angles)
        self.y=np.sin(2*self.angles)
        # print(type(self.x))
        self.v=0.5
        # msg = TrajectorySetpoint()
        # num_points=200
        # angles=np.linspace(0,2*np.pi,num_points)
        # # Shape 8
        # x=np.cos(angles)
        # y=np.sin(2*angles)
        # v=0.5
        # x_waypoints=[0,1,2]
        # y_waypoints=[1,3,2]
        # f=CubicSpline(x_waypoints,y_waypoints,bc_type='natural')
        # x_new=np.linspace(0,3,10)
        # y_new=f(x_new)
        # n=len(x_waypoints)
        # for i in range(n-1):
        #     f=CubicSpline([x_waypoints[i],x_waypoints[i+1]])
        # print(x_new)
        # print(y_new)
        plt.plot(self.x,self.y,'b')
        # plt.show()
        # plt.savefig("dummy_name.png")
        # Circle diameter=1
        # x=np.sin(angles)
        # y=np.cos(angles)
        # v=0.05
        # Any spline
        # x=
        self.count=0
        # count=0
        # Need to look into time syncronization in the future: Rahul

        # msg.timestamp = self.get_clock().now().to_msg()

        # msg = Pose()
        # while rclpy.ok():
        #     msg.position=[x[count], y[count], -1.0]
        #     msg.velocity=[v,v,v]
        #     time.sleep(0.1)
        #     count+=1
        #     print(count)
        #     if(count==num_points):
        #         msg.velocity=[0.0,0.0,0.0]
        #         break

        #     self.publisher.publish(msg)
        #     self.get_logger().info('Publishing setpoint')

        # print("Endpoint Reached")

    def publish_text(self):
        if self.count < self.num_points:
            self.msg.position = [self.x[self.count], self.y[self.count], -0.16]
            self.msg.velocity = [self.v, self.v, self.v]
            self.publisher.publish(self.msg)
            self.get_logger().info('Publishing setpoint %d' % self.count)
            self.count += 1
        else:
            self.msg.velocity = [0.0, 0.0, 0.0]
            self.publisher.publish(self.msg)
            self.get_logger().info('Endpoint Reached')
            print(type(received_x))
            print(received_x)
            cvt_received_x=np.asarray(received_x)
            cvt_received_y=np.asarray(received_y)
            print(cvt_received_x)
            plt.plot(cvt_received_x,cvt_received_y,'r')
            plt.show()
            plt.savefig("dummy_name_1.png")
            print("Image saved")
            self.timer.cancel()  # Stop the timer when all points are published


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

