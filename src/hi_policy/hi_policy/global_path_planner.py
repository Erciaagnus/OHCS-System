#/usr/bin/env pyhon3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
import json
from ament_index_python.packages import get_package_share_directory
import os
from collections import defaultdict
import heapq
from std_msgs.msg import ColorRGBA
from parking_world.rail_visualizer import RailNode, RailSegment, RailMap
def load_rail_map_from_json():
    pkg_path = get_package_share_directory('parking_world')
    json_path = os.path.join(pkg_path, 'maps', 'rail_map.json')
    with open(json_path, 'r') as f:
        data = json.load(f)
        # Convert dict to RailMap
    rail_map = RailMap()
    for n in data["nodes"]:
        rail_map.nodes[n["id"]] = RailNode(n["id"], n["x"], n["y"])
    for s in data["segments"]:
        rail_map.segments[s["id"]] = RailSegment(
            s["id"], s["start_node"], s["end_node"], s["points"], s["type"]
        )
    return rail_map

def dijkstra(graph, start_id, goal_id):
        queue = [(0, start_id, [])]
        visited = set()
        while queue:
            cost, current, path = heapq.heappop(queue)
            if current in visited:
                continue
            path = path + [current]
            if current == goal_id:
                return path
            visited.add(current)
            for neighbor, edge_cost in graph[current]:
                if neighbor not in visited:
                    heapq.heappush(queue, (cost + edge_cost, neighbor, path))
        return None
    #TODO(5) : segment.points -> path creating
class GlobalPlanner():
    def __init__(self):
        self.rail_map = load_rail_map_from_json()
        self.graph = self.build_graph(self.rail_map)

    def build_graph(self, rail_map): #node_id : list of (neighbor_id, cost)
        graph = defaultdict(list)
        for seg in rail_map.segments.values():
            s, e = seg.start_node, seg.end_node
            pts = seg.points
            cost = 0.0
            for i in range(len(pts)-1):
                dx = pts[i+1][0] - pts[i][0]
                dy = pts[i+1][1] - pts[i][1]
                cost += (dx**2 + dy**2) ** 0.5
            graph[s].append((e, cost))
            graph[e].append((s, cost))
        return graph
    #TODO(1) : Input the Start/Goal Coordinate (x,y)
    #TODO(2) : rail_map.json -> Node-Edge Graph
    #TODO(3) : Find the nearest node ( Coordinate -> Node ID)
    def find_closest_node(self, x,y, rail_map):
        min_dist = float('inf')
        closest_id = None
        for node in rail_map.nodes.values():
            dx = node.x - x
            dy = node.y - y
            dist = dx**2 + dy**2
            if dist < min_dist:
                min_dist = dist
                closest_id = node.id
        return closest_id
    #TODO(4) : Graph Search (Dijkstra or A*)

    def expand_path_nodes_to_points(self, path_node_ids, rail_map):
        points = []
        for i in range(len(path_node_ids) - 1):
            s, e = path_node_ids[i], path_node_ids[i+1]
            for seg in rail_map.segments.values():
                if (seg.start_node == s and seg.end_node == e) or (seg.start_node == e and seg.end_node == s):
                    if points and points[-1] == seg.points[0]:
                        points.extend(seg.points[1:])  # 중복 제거
                    else:
                        points.extend(seg.points)
                    break
        return points
    #TODO(6) : nav_msgs/Path or custom format publish or return
    def plan(self, start_xy, goal_xy):
        start_id = self.find_closest_node(*start_xy, self.rail_map)
        goal_id = self.find_closest_node(*goal_xy, self.rail_map)
        path_nodes = dijkstra(self.graph, start_id, goal_id)
        if not path_nodes:
            return []
        return self.expand_path_nodes_to_points(path_nodes, self.rail_map)

# class PathPublisher(Node):
#     def __init__(self):
#         super().__init__('global_path_planner')
#         self.publisher = self.create_publisher(Path, '/global_path', 10)
#         self.planner = GlobalPlanner()
#         self.timer = self.create_timer(2.0, self.publish_path)

#     def publish_path(self):
#         start_xy = (10.0, -5.0)  # Example start
#         goal_xy = (30.0, -25.0)  # Example goal
#         path_pts = self.planner.plan(start_xy, goal_xy)

#         path_msg = Path()
#         path_msg.header.frame_id = "map"
#         for pt in path_pts:
#             pose = PoseStamped()
#             pose.pose.position.x = pt[0]
#             pose.pose.position.y = pt[1]
#             pose.pose.position.z = 0.0
#             path_msg.poses.append(pose)
#         self.publisher.publish(path_msg)
class PathPublisher(Node):
    def __init__(self):
        super().__init__('global_path_planner')
        self.publisher = self.create_publisher(Path, '/global_path', 10)
        self.marker_pub = self.create_publisher(Marker, '/global_path_marker', 10)  # <-- 이 줄 추가 필요!

        self.planner = GlobalPlanner()

        # ⭐ 처음 한 번만 경로 계산
        start_xy = (10.0, -5.0)
        goal_xy = (30.0, -25.0)
        path_pts = self.planner.plan(start_xy, goal_xy)

        # ✅ Path 메시지 한 번 구성
        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"
        for pt in path_pts:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            pose.pose.position.z = 0.05
            self.path_msg.poses.append(pose)
        self.marker_msg = self.build_marker(path_pts)
        # 주기적으로 같은 경로 퍼블리시
        self.timer = self.create_timer(1.0, self.publish_path)

    def build_marker(self, points):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "global_path_marker"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.2  # ✅ 선 굵기 조절
        marker.color = ColorRGBA(r=1.0, g=0.2, b=0.0, a=1.0)  # 주황색

        for pt in points:
            marker.points.append(Point(x=pt[0], y=pt[1], z=0.05))  # 위로 띄움
        return marker

    def publish_path(self):
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.marker_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.path_msg)
        self.marker_pub.publish(self.marker_msg)
# --- Main Entry Point ---
def main(args=None):
    rclpy.init(args=args)

    # 1. 경로 계획 테스트
    planner = GlobalPlanner()
    start_xy = (10.0, -5.0)
    goal_xy = (30.0, -25.0)
    path = planner.plan(start_xy, goal_xy)

    if path:
        print(f"총 경로 길이: {len(path)}점")
        #for pt in path:
            #print(f"{pt}")
    else:
        print("❌ 경로를 찾을 수 없습니다.")

    # 2. ROS 퍼블리셔 실행
    node = PathPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()