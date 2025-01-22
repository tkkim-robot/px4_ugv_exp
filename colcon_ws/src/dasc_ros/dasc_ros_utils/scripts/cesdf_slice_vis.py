#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np
import cv2

from nvblox_msgs.msg import DistanceMapSlice
from math import floor
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from px4_msgs.msg import TrajectorySetpoint
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped


def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class ObsMapProcNode(Node):
    def __init__(self):
        super().__init__('obs_map_processor')
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscriber_map_slice = self.create_subscription(
            DistanceMapSlice, 
            '/nvblox_node/map_slice', 
            self.nvblox_map_callback, 
            qos_profile)
        self.subscriber_odom = self.create_subscription(
            PoseStamped,
            '/visual_slam/tracking/vo_pose',
            self.odom_callback,
            qos_profile)
        #self.subscriber_cmd = self.create_subscription(TrajectorySetpoint, '/px4_1/fmu/in/trajectory_setpoint', self.cmd_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.joystick_subscriber = self.create_subscription(
            Joy,
            'joystick',
            self.joystick_callback,
            qos_profile)

        self.publisher = self.create_publisher(TrajectorySetpoint, '/px4_7/fmu/in/trajectory_setpoint', 10)

        self.obs_msg = Float32MultiArray()
        
        # Parameters you can tune
        self.epsilon = 0.075                      # Threshold for obstacles
        self.submap_size_m = 5.0               # Size of the square region around the robot (meters)
        self.robot_radius = 0.17                 # Robot radius (meters)
        self.padding_radius = 0.15                 # Robot radius (meters)
        self.padding_radius_pixels = 2            # Radius of the robot marker (in pixels, after submap extraction)
        self.orientation_length_pixels = 20     # Length of the orientation line (in pixels, can be tied to speed)
        self.command_arrow_length = 50     
        
        # Color definitions (BGR)
        self.color_unknown = (127, 127, 127)    # Gray
        self.color_free = (255, 255, 255)       # White
        self.color_obstacle = (0, 0, 0)         # Black
        self.color_robot = (255, 0, 0)          # Blue circle
        self.color_orientation = (0, 0, 255)    # Red line
        self.color_command_arrow = (0, 255, 0)  # Green arrow

        self.L = 0.3302 # Wheel base in m

        # initialize submap
        self.map_slice = np.full((100, 100, 3), 0, dtype=np.uint8)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.map_resolution = 0.1
        self.map_origin_x = 0
        self.map_origin_y = 0

        self.upsample_factor = 5
        self.safety_check_time_step = 0.1
        self.safety_check_duration = 1.0
        # cmd from keyboard
        self.lin_vel = 0.0
        self.yaw_rate = 0.0
        self.MAX_SPEED = 0.5
        self.MAX_STEER = 3.0
        self.gear = 1
        print("ObsMapProcNode initialized.")

    def joystick_callback(self, msg):
        control_speed = 0.
        control_steer = 0.
        # if args.joy:
        # CM vehicle control
        UserTakeControl = msg.buttons[6]  # LB Button
        FullBrake = msg.buttons[7]
        FrontGear = msg.buttons[4]
        BackGear = msg.buttons[0]

        if FrontGear:
            self.gear = 1
        elif BackGear:
            self.gear = -1

        if UserTakeControl:
            control_speed = (msg.axes[4] + 1) / 2  # Right trigger
            control_brake = (msg.axes[0] + 1) / 2  # Left trigger
            control_steer = msg.axes[0]  # Left stick

            control_speed *= self.MAX_SPEED
            control_steer *= self.MAX_STEER

        # Send Actions to Truckmaker
        self.lin_vel = control_speed * self.gear
        self.yaw_rate = control_steer

        if FullBrake:
            self.lin_vel = 0.0


        # #FIXME: temporal
        # self.publish_wheel_vel(self.lin_vel, self.yaw_rate)

    def publish_wheel_vel(self, lin_vel, yaw_rate):

        msg = TrajectorySetpoint()

        self.max_vel = 0.8 # spec of motor
        self.min_vel = 0.02

        if lin_vel > 0:
            lin_vel = np.clip(lin_vel, self.min_vel, self.max_vel)
        elif lin_vel < 0:
            lin_vel = np.clip(lin_vel, -self.max_vel, -self.min_vel)

        v_l = -lin_vel - yaw_rate * self.L / 2
        v_r = -lin_vel + yaw_rate * self.L / 2
        # if absolute value is smaller than 0.2, then set it to 0.2 with the sign
        if abs(v_l) < self.min_vel:
            v_l = self.min_vel if v_l > 0 else -self.min_vel
        if abs(v_r) < self.min_vel:
            v_r = self.min_vel if v_r > 0 else -self.min_vel

        print("Wheel vel: ", [v_l, v_r])
        msg.raw_mode = True
        msg.cmd = [v_l, -v_r, 0.0, 0.0] # right motor is flipped
        self.publisher.publish(msg)

    def odom_callback(self, msg):
        """
        Callback to handle odometry, extract a 5x5 meter submap around the robot,
        upsample it, then visualize the robot with a blue circle and heading
        line in red.
        """
        # --- 1. Extract pose, orientation, etc. ---
        pose = msg.pose.position
        orientation_quat = msg.pose.orientation
        # velocity = msg.twist.twist.linear
        
        # Convert quaternion to Euler angles (roll, pitch, yaw)
        roll, pitch, yaw = euler_from_quaternion(orientation_quat)

        self.robot_x = pose.x
        self.robot_y = pose.y
        self.robot_yaw = yaw


        # --- 2. Convert robot global coords to pixel coords in the *full* map ---
        robot_px, robot_py = self.cvt_global_to_pixel(
            pose.x,
            pose.y,
            self.map_origin_x,        # map origin in global coords (meters)
            self.map_origin_y,        # map origin in global coords (meters)
            self.map_resolution       # meters per pixel
        )

        # Safety checks: if self.map_slice is None or has zero size, return
        if self.map_slice is None or self.map_slice.size == 0:
            self.get_logger().warn("Map slice not yet available or empty.")
            return
        
        # The full map is a color image stored in self.map_slice (BGR or grayscale).
        # Make a local copy to avoid modifying the class member in-place:
        full_map = self.map_slice.copy()
        h_full, w_full = full_map.shape[:2]  # If color, shape = (H, W, 3)
        
        # --- 3. Compute submap bounds for ~5 meters around the robot ---
        #     half_size_px is half the number of pixels corresponding to ~2.5m each way
        half_size_m = self.submap_size_m / 2.0
        half_size_px = int(round(half_size_m / self.map_resolution))  # radius in pixels
        
        x_min = max(0, robot_px - half_size_px)
        x_max = min(w_full, robot_px + half_size_px)
        y_min = max(0, robot_py - half_size_px)
        y_max = min(h_full, robot_py + half_size_px)

        # If no valid region, return
        if x_min >= x_max or y_min >= y_max:
            self.get_logger().warn("Submap extraction area invalid (out of image bounds).")
            return

        # Extract submap
        submap = full_map[y_min:y_max, x_min:x_max].copy()
        
        # --- 4. Upsample the submap (increase resolution) ---
        #     For example, upsample by factor=5. You can make this a parameter.
        submap_upsampled = cv2.resize(
            submap,
            None,    # no explicit size
            fx=self.upsample_factor,
            fy=self.upsample_factor,
            interpolation=cv2.INTER_NEAREST
        )

        # --- 5. Compute the robot’s new center within the upsampled submap ---
        #     (The robot’s submap-local coords before upsampling:)
        submap_robot_px = (robot_px - x_min)
        submap_robot_py = (robot_py - y_min)

        #     (Now multiply by the upsample factor:)
        up_robot_px = submap_robot_px * self.upsample_factor
        up_robot_py = submap_robot_py * self.upsample_factor

        # --- 6. Draw robot’s circle and heading in the upsampled submap ---
        # Draw circle
        cv2.circle(
            submap_upsampled,
            (up_robot_px, up_robot_py),
            self.padding_radius_pixels * self.upsample_factor,  # scale radius if desired
            self.color_robot,  # (B, G, R) for blue
            -1
        )

        # --- Visualize command direction as an arrow ---
        # Scale the linear velocity to a length for visualization
        arrow_length = int(self.command_arrow_length * np.abs(self.lin_vel)) * np.sign(self.lin_vel)

        # Compute the arrow end coordinates
        cmd_end_x = up_robot_px + int(arrow_length * np.cos(yaw + self.yaw_rate * 0.3))
        cmd_end_y = up_robot_py + int(arrow_length * np.sin(yaw + self.yaw_rate * 0.3))

        # Draw the arrow on the submap
        cv2.arrowedLine(
            submap_upsampled,
            (up_robot_px, up_robot_py),  # Arrow start at robot center
            (cmd_end_x, cmd_end_y),     # Arrow end based on lin_vel and yaw_rate
            self.color_command_arrow,   # Color (e.g., green)
            2,                          # Thickness
            tipLength=0.3               # Length of the arrowhead tip
        )

        # Heading line
        line_end_x = up_robot_px + int(self.orientation_length_pixels * np.cos(yaw))
        line_end_y = up_robot_py + int(self.orientation_length_pixels * np.sin(yaw))
        cv2.line(
            submap_upsampled,
            (up_robot_px, up_robot_py),
            (line_end_x, line_end_y),
            self.color_orientation,  # e.g., red
            2
        )

        # apply safety filter and draw intermediate points
        self.safe_lin_vel, self.safe_yaw_rate = self.safety_filter(submap, submap_upsampled, (x_min, x_max, y_min, y_max))
        self.publish_wheel_vel(self.safe_lin_vel, self.safe_yaw_rate)

        # --- 7. (Optional) Rotate the submap so “up” is the desired orientation ---
        # For example, rotate 90° CCW:
        submap_final = cv2.rotate(submap_upsampled, cv2.ROTATE_90_COUNTERCLOCKWISE)
        # flip in horizontally
        submap_final = cv2.flip(submap_final, 1)
        # --- 8. Save (or publish) the final submap ---
        cv2.imwrite("/root/ros_ws/truncated_submap.png", submap_final)
        
        # --- 9. Log or debug prints ---
        # self.get_logger().info(
        #     f"Saved upsampled submap around robot at {pose.x:.2f}, {pose.y:.2f}, yaw={yaw:.2f}"
        # )

    def nvblox_map_callback(self, msg):
        """Callback for processing the ESDF map slice from nvblox."""
        map_slice = np.array(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        width, height = msg.width, msg.height
        self.map_resolution = msg.resolution
        unknown_val = msg.unknown_value

        # Convert the map into a mask for obstacles vs. free vs. unknown
        #   - unknown -> gray
        #   - free -> white
        #   - obstacle -> black
        obstacle_mask = (map_slice < self.epsilon) & (map_slice != unknown_val)
        known_mask = (map_slice != unknown_val)

        # Initialize a color map as all gray (unknown)
        color_map = np.full((height, width, 3), self.color_unknown, dtype=np.uint8)
        # Mark obstacles (black) wherever map_slice < epsilon
        color_map[obstacle_mask] = self.color_obstacle
        # Mark free cells (white) in all known but non-obstacle areas
        free_mask = known_mask & (~obstacle_mask)
        color_map[free_mask] = self.color_free

        self.map_origin_x = msg.origin.x
        self.map_origin_y = msg.origin.y

        map_slice = color_map

        # If submap is empty for some reason (out of bounds), exit early
        if map_slice.size == 0:
            print("Submap is empty. Robot may be out of map bounds.")
            return
        # Save the submap image for visualization
        self.map_slice = map_slice
        
        # You could publish or do more advanced visualization here instead of just saving.

    def preprocess_map_with_robot_radius(self, submap):
        """
        Preprocess the map by dilating obstacles to account for the robot's radius,
        considering free space is white ([255, 255, 255]).
        """
        # Convert the map to a binary format (0 = free space, 1 = obstacle/unknown)
        binary_map = np.where(submap == 255, 0, 1).astype(np.uint8)

        # Convert robot radius to pixels
        robot_radius_px = int(self.padding_radius / self.map_resolution)

        # Use a circular structuring element to dilate obstacles
        structuring_element = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, 
            (2 * robot_radius_px, 2 * robot_radius_px)
        )

        # Dilate the binary map
        dilated_binary_map = cv2.dilate(binary_map, structuring_element)

        # Convert back to the original map format (white for free space)
        return np.where(dilated_binary_map == 1, 0, 255).astype(np.uint8)

    def safety_filter(self, submap, submap_upsample, submap_min_max):
        """
        Safety filter to check if the robot's current command will lead to a collision
        with an obstacle. If a collision is detected, print a warning message.
        Visualizes the intermediate states onto the map.
        """
        # Time step for forward propagation
        dt = self.safety_check_time_step  # Time step for discrete propagation
        total_time = self.safety_check_duration  # Total forward propagation time
        num_steps = int(total_time / dt)

        # Robot's initial state (position and heading)
        robot_x, robot_y = self.robot_x, self.robot_y  # Current robot global position
        yaw = self.robot_yaw  # Current yaw (orientation)

        if self.gear == -1:
            # change the starting location to the back of the robot
            robot_x -= self.robot_radius * np.cos(yaw)
            robot_y -= self.robot_radius * np.sin(yaw)

        x_min, x_max, y_min, y_max = submap_min_max

        dilated_submap = self.preprocess_map_with_robot_radius(submap)
        self.safe_yaw_rate = self.yaw_rate

        # Iterate through time steps to predict the trajectory
        for step in range(num_steps):
            # Propagate position based on current linear velocity and yaw rate
            robot_x += self.lin_vel * np.cos(yaw) * dt
            robot_y += self.lin_vel * np.sin(yaw) * dt
            yaw += self.yaw_rate * dt

            # Convert global robot coordinates to pixel coordinates on the map
            pixel_x, pixel_y = self.cvt_global_to_pixel(
                robot_x, robot_y, 
                self.map_origin_x, self.map_origin_y, self.map_resolution
            )

            # Convert pixel coordinates to submap coordinates
            pixel_x -= x_min
            pixel_y -= y_min

            # Draw a red circle at the collision point
            up_pixel_x = int((pixel_x) * self.upsample_factor)
            up_pixel_y = int((pixel_y) * self.upsample_factor)

            # Check if the position is an obstacle or unknown
            if np.sum(dilated_submap[pixel_y, pixel_x]) != 255*3 and step == num_steps-1:  # in the original submap, 255 is free, but in dilated map, 0 is free
                print("Collision detected with an obstacle at step", step + 1)

                self.safe_lin_vel = 0.0
                self.safe_yaw_rate /= 1.5 # allow rotating in place

                cv2.circle(
                    submap_upsample,
                    (up_pixel_x, up_pixel_y),
                    5,  # Radius of the circle
                    (0, 0, 255),  # Red color
                    -1
                )
                break

            # Visualize the intermediate state with a green circle
            cv2.circle(
                submap_upsample,
                (up_pixel_x, up_pixel_y),
                2,  # Radius of the circle
                (0, 255, 100),  # Green color
                -1
            )
        else:
            #print("No collisions detected. Command is safe.")
            self.safe_lin_vel = self.lin_vel
            self.safe_yaw_rate = self.yaw_rate

        dilated_submap = cv2.rotate(dilated_submap, cv2.ROTATE_90_COUNTERCLOCKWISE)

        # --- 8. Save (or publish) the final submap ---
        cv2.imwrite("/root/ros_ws/src/dilated_submap.png", dilated_submap)

        return self.safe_lin_vel, self.safe_yaw_rate

    def cvt_global_to_pixel(self, global_x, global_y, map_origin_x, map_origin_y, resolution):
        """
        Convert a global coordinate (in meters) to pixel coordinates relative to 
        the map's origin (also in global coordinates), using the map's resolution 
        (meters per pixel).
        """
        delta_x = global_x - map_origin_x
        delta_y = global_y - map_origin_y
        
        pixel_x = int(round(delta_x / resolution))
        pixel_y = int(round(delta_y / resolution))
        
        return pixel_x, pixel_y

def main(args=None):
    rclpy.init(args=args)
    node = ObsMapProcNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
