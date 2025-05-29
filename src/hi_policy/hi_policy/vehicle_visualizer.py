#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

class ChargerVisualizer(Node):
    def __init__(self):
        super().__init__('charger_visualizer')
        self.publisher_ = self.create_publisher(Marker, 'charger_marker', 10)
        self.timer = self.create_timer(0.5, self.publish_marker)
        self.charger_position = [0.5, -4.0]

    def publish_marker(self):
        marker= Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "chargers"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = self.charger_position[0]
        marker.pose.position.y = self.charger_position[1]
        marker.pose.position.z = 0.4
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.4
        marker.scale.y = 0.7
        marker.scale.z = 1.0

        marker.color.r = 0.53
        marker.color.g = 0.81
        marker.color.b = 0.92
        marker.color.a = 1.0

        self.publisher_.publish(marker)
def main(args=None):
    rclpy.init(args=args)
    node=ChargerVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()