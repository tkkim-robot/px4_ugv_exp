#!/usr/bin/env python3
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.node import Node
from px4_msgs.msg import TrajectorySetpoint
from std_msgs.msg import Float32MultiArray
import time
import numpy as np

class SetpointAssignerNode(Node):
    def __init__(self):
        super().__init__('setpoint_assigner')
        self.publisher = self.create_publisher(TrajectorySetpoint, '/px4_1/fmu/in/trajectory_setpoint', 10)
        self.ctrl_vel_subscription = self.create_subscription(
            Float32MultiArray,
            'ctrl_vel',
            self.ctrl_vel_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        self.msg = TrajectorySetpoint()
        self.control_flag = False
        self.acc_x = 0.0  # Linear acceleration in m/s^2
        self.yaw_rate = 0  # Yaw rate in rad/sec ; +ve moves left, -ve moves right
        self.start_time = time.time()
        self.current_time = self.start_time
        self.L = 0.55 #0.3302  # Wheel base in m

        # to store previous vel_x of the robot
        self.vel_longitudinal = 0.0
        self.max_vel = 0.8 # spec of motor
        self.min_vel = 0.02

        self.timer = self.create_timer(0.05, self.timer_callback)

    def ctrl_vel_callback(self, msg):
        self.control_flag = True

        self.acc_x = msg.data[0] # desired
        self.yaw_rate = msg.data[1] # desired
        if self.acc_x == 0.0 and self.yaw_rate == 0.0:
            self.control_flag = False
        #self.get_logger().info(f'Received control inputs - acc_x: {self.acc_x}, yaw_rate: {self.yaw_rate}')

    def timer_callback(self):
        if self.control_flag:
            self.current_time = time.time()
            dt = 0.05
            self.msg.raw_mode = True

            self.vel_longitudinal += self.acc_x * dt
            print("Acc: ", self.acc_x)
            print("Yaw: ", self.yaw_rate)
            print("dt: ", dt)
            print("Long vel before clip: ",self.vel_longitudinal)

            self.vel_longitudinal = np.clip(self.vel_longitudinal, 0.0, self.max_vel)
            print("Long vel after clip : ", self.vel_longitudinal)

            v_l = self.vel_longitudinal - self.yaw_rate * self.L / 2
            v_r = self.vel_longitudinal + self.yaw_rate * self.L / 2
            # if absolute value is smaller than 0.2, then set it to 0.2 with the sign
            if abs(v_l) < self.min_vel:
                v_l = self.min_vel if v_l > 0 else -self.min_vel
            if abs(v_r) < self.min_vel:
                v_r = self.min_vel if v_r > 0 else -self.min_vel
        else:
            self.msg.raw_mode = True
            v_l = 0.0
            v_r = 0.0

        print("Wheel vel: ", [v_l, v_r])
        self.msg.cmd = [v_l, -v_r, 0.0, 0.0] # right motor is flipped
        self.publisher.publish(self.msg)
        # print(dt)
        self.start_time = self.current_time

def main(args=None):
    rclpy.init(args=args)
    setpoint_assigner_node = SetpointAssignerNode()
    rclpy.spin(setpoint_assigner_node)
    setpoint_assigner_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
