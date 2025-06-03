#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from hlc_interfaces.msg import ChargerState
from typing import List, Tuple, Dict
from parking_world.rail_visualizer import RailNode, RailSegment, RailMap
class ChargerVisualizer(Node):
    def __init__(self):
        super().__init__('charger_visualizer')
        self.publisher_ = self.create_publisher(MarkerArray, 'charger_marker', 10)
        self.create_subscription(ChargerState, '/charger_states', self.charger_callback, 10)
        self.charger_positions = {}  # key: charger_id, value: [x, y]
        self.timer = self.create_timer(0.5, self.publish_markers)

    def charger_callback(self, msg):
        self.charger_positions[msg.charger_id] = [msg.location.position.x, msg.location.position.y]
    def publish_markers(self):
        marker_array = MarkerArray()
        for i, (charger_id, pos) in enumerate(self.charger_positions.items()):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "chargers"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            marker.pose.position.x = pos[0]
            marker.pose.position.y = pos[1]
            marker.pose.position.z = 0.4
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.4
            marker.scale.y = 0.7
            marker.scale.z = 1.0

            marker.color.r = 0.53
            marker.color.g = 0.81
            marker.color.b = 0.92
            marker.color.a = 1.0

            marker_array.markers.append(marker)
        self.publisher_.publish(marker_array)
class GoalVisualizer(Node):
    def __init__(self):
        super().__init__('user_visualizer')
        self.publisher = self.create_publisher(Marker, '/user_markers', 10)

    def publish_goals(self, goal_nodes: str, rail_map: RailMap):
        node = rail_map.nodes.get(goal_nodes)
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "goal_nodes"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(node.x)
        marker.pose.position.y = float(node.y)
        marker.pose.position.z = 2.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 2.0
        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)

        self.publisher.publish(marker)
    def remove_goal(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "goal_nodes"
        marker.id = 0
        marker.action = Marker.DELETE
        self.publisher.publish(marker)
def main(args=None):
    rclpy.init(args=args)
    node = ChargerVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

