import rclpy 
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import VehicleLocalPosition



def callback(node, msg):

    # print(f"now: {node.get_clock().now()} stamp: {msg.timestamp} stampsample: {msg.timestamp_sample}")

    now = node.get_clock().now().nanoseconds * 1.0 / 1000
    sample = msg.timestamp_sample

    print(f"{now - sample}")



rclpy.init()

node = Node("debug")


node.create_subscription(VehicleLocalPosition, 
        "/px4_1/fmu/out/vehicle_local_position",
        lambda msg: callback(node, msg),
        qos_profile_sensor_data
        )

rclpy.spin(node)




