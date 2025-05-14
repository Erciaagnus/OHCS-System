#/usr/bin/env pyhon3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import json
from ament_index_python.packages import get_package_share_directory
import os

# --- RailMap 관련 클래스 (이미 정의되어 있다고 가정) ---
class RailNode():
    def __init__(self, id, x, y):
        self.id = id
        self.x = x
        self.y = y
        self.connected_segments = []

class RailSegment():
    def __init__(self, id, start_node, end_node, points, segment_type='straight'):
        self.id = id
        self.start_node = start_node
        self.end_node = end_node
        self.points = points or []
        self.segment_type = segment_type

class RailMap():
    def __init__(self):
        self.nodes = {}
        self.segments = {}

    def add_node(self, x, y, name):
        self.nodes[name] = RailNode(name, x, y)
    
    def add_segment(self, start_id, end_id, typ, pts):
        rid = f"E{len(self.segments)+1}"
        seg = RailSegment(rid, start_id, end_id, pts, typ)
        self.segments[rid] = seg
def load_rail_map_from_json(path: str) -> RailMap:
    with open(path, 'r') as f:
        data = json.load(f)

    rail_map = RailMap()
    for n in data["nodes"]:
        rail_map.add_node(n["x"], n["y"], n["id"])
    for s in data["segments"]:
        rail_map.add_segment(s["start_node"], s["end_node"], typ=s["type"], pts=s["points"])
    return rail_map
class RailVisualizer(Node):
    def __init__(self):
        super().__init__('rail_visualizer')
        self.publisher = self.create_publisher(MarkerArray, 'rail_marker_array', 10)
        self.timer = self.create_timer(1.0, self.publish_markers)
        package_path = get_package_share_directory('parking_world')
        json_path = os.path.join(package_path, 'maps', 'rail_map.json')
        self.rail_map = load_rail_map_from_json(json_path)
        self.markers_published = False

    def publish_markers(self):
        if self.markers_published:
            return

        marker_array = MarkerArray()

        # 1. 선 마커
        rail_line = Marker()
        rail_line.header.frame_id = "map"
        rail_line.header.stamp = self.get_clock().now().to_msg()
        rail_line.ns = "rail"
        rail_line.id = 0
        rail_line.type = Marker.LINE_LIST
        rail_line.action = Marker.ADD
        rail_line.scale.x = 0.05
        rail_line.color.r = 1.0
        rail_line.color.g = 0.5
        rail_line.color.b = 0.0
        rail_line.color.a = 1.0

        for segment in self.rail_map.segments.values():
            for i in range(len(segment.points) - 1):
                p1 = segment.points[i]
                p2 = segment.points[i + 1]
                rail_line.points.append(Point(x=p1[0], y=p1[1], z=0.02))
                rail_line.points.append(Point(x=p2[0], y=p2[1], z=0.02))
        marker_array.markers.append(rail_line)

        # 2. 텍스트 마커
        text_id = 1000
        for node in self.rail_map.nodes.values():
            txt = Marker()
            txt.header.frame_id = "map"
            txt.header.stamp = self.get_clock().now().to_msg()
            txt.ns = "labels"
            txt.id = text_id
            text_id += 1
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.scale.z = 0.25
            txt.color.r = 1.0
            txt.color.g = 1.0
            txt.color.b = 0.2
            txt.color.a = 1.0
            txt.pose.position.x = node.x
            txt.pose.position.y = node.y
            txt.pose.position.z = 0.7
            txt.text = node.id
            marker_array.markers.append(txt)
        # --- 3. 노드 점 마커 (원)
        node_id = 2000
        for node in self.rail_map.nodes.values():
            dot = Marker()
            dot.header.frame_id = "map"
            dot.header.stamp = self.get_clock().now().to_msg()
            dot.ns = "node_dots"
            dot.id = node_id
            node_id += 1
            dot.type = Marker.CYLINDER
            dot.action = Marker.ADD
            dot.scale.x = 0.3
            dot.scale.y = 0.3
            dot.scale.z = 0.05
            dot.color.r = 1.0
            dot.color.g = 0.0
            dot.color.b = 0.0
            dot.color.a = 1.0
            dot.pose.position.x = node.x
            dot.pose.position.y = node.y
            dot.pose.position.z = 0.01
            marker_array.markers.append(dot)
        self.publisher.publish(marker_array)
def main(args=None):
    rclpy.init(args=args)
    node = RailVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()