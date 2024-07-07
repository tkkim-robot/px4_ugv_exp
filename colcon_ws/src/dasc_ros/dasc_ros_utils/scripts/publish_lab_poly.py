#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PolygonStamped, Point32


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('lab_polygon_publisher')

        self.publisher_ = self.create_publisher(PolygonStamped, 'lab_polygon', 10)
        
        msg = PolygonStamped()

        msg.header.frame_id = "vicon/world"

        inner_xs = [-3.2, -3.2, -2.0, -2.0, 2.6, 2.6, 3.0, 3.0, -3.2 ]
        inner_ys = [-2.1, 1.9, 1.9, 2.4, 2.4, 1.1, 1.1, -2.1, -2.1]
        N = len(inner_xs)

        for i in range(N):
            p = Point32()
            p.x = inner_xs[i]
            p.y = inner_ys[i]
            p.z = 0.0
            msg.polygon.points.append(p)


        N = 5
        for i in range(5):
            self.publisher_.publish(msg)
            time.sleep(1)
            self.get_logger().info(f"publishing lab poly [{i}/{N}]...")


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
