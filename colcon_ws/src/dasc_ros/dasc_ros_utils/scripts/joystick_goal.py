import numpy as np


import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data as qos_sensor_data

from std_msgs.msg import String
from sensor_msgs.msg import Joy
from px4_msgs.msg import TrajectorySetpoint, VehicleLocalPosition


class JoyGoal(Node):

    def __init__(self):
        super().__init__('joystick_goal')

        ## publishers
        self.setpoint_publisher = self.create_publisher(TrajectorySetpoint, 'px4_1/fmu/in/trajectory_setpoint', 10)


        ## subscribers
        self.create_subscription(Joy, "joy", self.joy_callback, 10)
        self.create_subscription(VehicleLocalPosition, "px4_1/fmu/out/vehicle_local_position", self.pos_callback, qos_sensor_data) 


        ## timer 
        self.dt = 0.01  # seconds
        self.timer = self.create_timer(self.dt, self.timer_callback)

        # variables 
        self.current_pos = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        self.goal = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        self.goal_vel = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        self.goal_yaw = np.pi/2
        self.got_joystick = False
        self.got_position = False


        # axis mappings
        self.FORWARD_CH = 3
        self.FORWARD_SCALE = 1
        self.RIGHT_CH = 2
        self.RIGHT_SCALE = -1
        self.UP_CH = 1
        self.UP_SCALE = -0.2 # move it up and down more slowly
        self.YAW_CH = 0
        self.YAW_SCALE = -2

        self.RESET_CH = 1
        self.TAKEOFF_CH = 4
        self.LAND_CH = 0

    def  pos_callback(self, msg):
        if not (msg.xy_valid and msg.z_valid and msg.heading_good_for_control):
            return

        self.current_pos = np.array([msg.x, msg.y, msg.z], dtype=np.float32)
        self.current_yaw = msg.heading

        if not self.got_position:
            self.goal = self.current_pos
            self.goal[2] = -1.0
            self.goal_yaw = self.current_yaw


        self.got_position = True


    def joy_callback(self, msg):
        # print(f"got joy msg: {msg.axes}")

        self.v_body_x = self.FORWARD_SCALE * msg.axes[self.FORWARD_CH]
        self.v_body_y = self.RIGHT_SCALE * msg.axes[self.RIGHT_CH]
        self.v_body_z = self.UP_SCALE * msg.axes[self.UP_CH]
        self.v_body_yaw = self.YAW_SCALE * msg.axes[self.YAW_CH]

        # print(f"vx: {v_body_x}, vy: {v_body_y}, yaw: {v_body_yaw}")
        self.got_joystick = True


        if msg.buttons[self.RESET_CH] == 1:
            self.goal = self.current_pos
            self.goal_yaw = self.current_yaw
            self.goal_vel = np.array([0,0,0], dtype=np.float32)

        if msg.buttons[self.TAKEOFF_CH] == 1:
            self.goal[2] = -1.0;

        if msg.buttons[self.LAND_CH] == 1:
            self.goal[2] = 0.0;

    def update_setpoint(self):

        vx = np.cos(self.goal_yaw) * self.v_body_x  - np.sin(self.goal_yaw) * self.v_body_y
        vy = np.sin(self.goal_yaw) * self.v_body_x  + np.cos(self.goal_yaw) * self.v_body_y
        vz = self.v_body_z

        self.goal += np.array([vx, vy, vz]) * self.dt
        self.goal_yaw += self.v_body_yaw * self.dt
        self.goal_vel = np.array([vx, vy, vz], dtype=np.float32)


    def timer_callback(self):
        if not(self.got_position and self.got_joystick):
            return

        msg = TrajectorySetpoint()
        
        ## update the target point
        self.update_setpoint()

        ## construct the message
        msg.position = self.goal
        msg.velocity = self.goal_vel
        msg.yaw = self.goal_yaw
        msg.raw_mode = False

        self.setpoint_publisher.publish(msg)
        # self.get_logger().info(f'publishing {msg.position}, {msg.yaw}')


def main(args=None):
    rclpy.init(args=args)

    node = JoyGoal()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
