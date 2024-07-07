import numpy as np


import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import String
from sensor_msgs.msg import Joy
from px4_msgs.msg import TrajectorySetpoint, VehicleLocalPosition
from dasc_msgs.msg import DITrajectory
from geometry_msgs.msg import PoseArray, Pose, Twist, Accel


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = np.cos(ai)
    si = np.sin(ai)
    cj = np.cos(aj)
    sj = np.sin(aj)
    ck = np.cos(ak)
    sk = np.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q



class JoyTraj(Node):

    def __init__(self):
        super().__init__('joystick_traj')

        ## publishers
        # self.setpoint_publisher = self.create_publisher(TrajectorySetpoint, 'px4_1/fmu/in/trajectory_setpoint', 10)
        self.nominal_publisher = self.create_publisher(DITrajectory, 'nominal_trajectory', 10)
        self.nominal_viz_publisher = self.create_publisher(PoseArray, 'nominal_trajectory/viz', 10)


        ## subscribers
        self.create_subscription(Joy, "joy", self.joy_callback, 10)
        self.create_subscription(VehicleLocalPosition, "px4_1/fmu/out/vehicle_local_position", self.pos_callback, qos_profile_sensor_data) 


        ## timer 
        self.dt = 0.1  # seconds
        self.timer = self.create_timer(self.dt, self.timer_callback)

        # variables 
        self.current_pos = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        self.goal = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        self.goal_vel = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        self.goal_yaw = 0.0 
        self.got_joystick = False
        self.got_position = True    # TODO: change back to false


        # axis mappings
        self.FORWARD_CH = 3
        self.FORWARD_SCALE = 1
        self.RIGHT_CH = 2
        self.RIGHT_SCALE = 1
        self.UP_CH = 1
        self.UP_SCALE = 0.2 # move it up and down more slowly
        self.YAW_CH = 0
        self.YAW_SCALE = 2

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

        ## update the target point
        self.update_setpoint()
        
        self.construct_traj_message()

    def construct_traj_message(self):

        msg = DITrajectory()

        msg.header.stamp = self.get_clock().now().to_msg() 
        msg.header.frame_id = "vicon/world"

        msg.dt = 0.05 # self.dt

        N = int(round(2.0 / msg.dt))

        # now construct a trajectory message into the future
        for i in range(N):
            pose = Pose()
            pose.position.x = self.goal[0] + i * self.goal_vel[0] * msg.dt
            pose.position.y = self.goal[1] + i * self.goal_vel[1] * msg.dt
            pose.position.z = self.goal[2] + i * self.goal_vel[2] * msg.dt
            q = quaternion_from_euler(0,0, self.goal_yaw)
            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]

            msg.poses.append(pose)

            twist = Twist()
            twist.linear.x = 1.0 * self.goal_vel[0]
            twist.linear.y = 1.0 * self.goal_vel[1]
            twist.linear.z = 1.0 * self.goal_vel[2]
            msg.twists.append(twist)


            msg.accelerations.append(Accel())

        self.nominal_publisher.publish(msg)

        vizMsg = PoseArray()
        vizMsg.header = msg.header
        vizMsg.poses = msg.poses
        self.nominal_viz_publisher.publish(vizMsg)


def main(args=None):
    rclpy.init(args=args)

    node = JoyTraj()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
