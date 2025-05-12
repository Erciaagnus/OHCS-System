#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Duration
import math
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import json
class RailNode():
    def __init__(self, id, x, y):
        self.id = id # Rail ID
        self.x = x
        self.y = y
        self.connected_segments = [] # Graph Rail

class RailSegment:
    def __init__(self, id, start_node, end_node, points = None, segment_type = 'straight'):
        self.id = id
        self.start_node = start_node
        self.end_node = end_node
        self.points = points or []
        self.segment_type = segment_type
class RailMap:
    def __init__(self):
        self.nodes = {}
        self.segments = {}
    def add_node(self, node):
        self.nodes[node.id] = node
    def add_segment(self, segment):
        self.segments[segment.id] = segment # Store the Rail Segment
        self.nodes[segment.start_node].connected_segments.append(segment.id) # Add the connecting info to starting node
    def to_json(self):
        return json.dumps({
            "nodes":[
                {"id": n.id, "x":n.x, "y":n.y} for n in self.nodes.values()
            ],
            "segments":[
                {
                    "id": s.id, # Segment ID
                    "start_node": s.start_node, # Start Node
                    "end_node" : s.end_node, # End Node
                    "points" : s.points, # Actual Rail Coordinate
                    "type" : s.segment_type # Straight/ Turning
                } for s in self.segments.values()
            ]
        }, indent=2)


    # Create Turning Points
def generate_turn(center, radius=2.0, angle_start=0, angle_end=math.pi/2, density=20):
    if center is None or not isinstance(center, (list, tuple)):
        raise ValueError("center는 [x, y] 형태의 리스트나 튜플이어야 합니다.")
    return [
        [center[0] + radius * math.cos(theta),
         center[1] + radius * math.sin(theta)]
        for theta in np.linspace(angle_start, angle_end, density)
    ]

    # Create Straight Points
def generate_straight(start, end, density=10):
    if start is None or end is None:
        raise ValueError("start 또는 end 좌표가 None입니다.")

    start = np.array(start)
    end = np.array(end)
    dist = np.linalg.norm(end - start)
    count = max(2, int(dist * density))
    return [list(p) for p in np.linspace(start, end, count)]

class RailVisualizer(Node):
    def __init__(self):
        super().__init__('rail_visualizer')
        self.publisher = self.create_publisher(Marker, 'rail_marker', 10)
        self.timer = self.create_timer(1.0, self.publish_markers)
        self.rail_map = self.build_rail_map()

    def build_rail_map(self):
        rail_map = RailMap() #nodes, segment
        # Parameters
        block_w = 6.9 # 2.3*3 -> 3x2 parking_lot
        block_h = 10.2 # Height of one block ( 5.1 * 2 )
        gap = 5.0 # Gap Space between blocks ( Pass )
        margin = 0.5 # Location far from the parking lot
        curve_r = 2.0 # Rail curve

        # Parking Num Parameter
        num_parking_col = 3
        num_parking_row = 2

        # Size of Parking
        width = 2.3
        height = 5.1

        segment_id = 1 # Set Segment ID ( RAIL )
        node_id = 1 # NODE NAME

        # for one block -> two horizontal Rail
        for row in range(num_parking_row):
            y = -row*(block_h + margin*2) + margin
            x1 = 0 - margin # start point of Rail
            prev_nid = None
            for col in range(num_parking_col):
                x = col*(width+0.1)
                nid = f"N{node_id}"
                rail_map.add_node(RailNode(nid, x, y))

                if prev_nid:
                    start = [rail_map.nodes[prev_nid].x, rail_map.nodes[prev_nid].y]
                    end = [x,y]
                    rail_map.add_segment(RailSegment(
                        id = f"R{segment_id}",
                        start_node = prev_nid,
                        end_node = nid,
                        points = generate_straight(start, end)
                    ))
                    segment_id += 1
                prev_nid = nid
                node_id += 1
            rail_map.add_node(RailNode(f"N1", x1, y))
        # for one block, two vertical rail
        prev_nid = node_id
        node_id += 1
        for col in range(2):
            x = col * (block_w + margin*2) - margin
            y1 = 0 + margin
            y_2 = block_h + margin*2
            nid = f"N{node_id}"
            rail_map.add_node(RailNode())
            rail_map.add_segment(RailSegment(
                id = f"R{segment_id}",
                start_node = prev_nid,
                end_node = nid,
                points = generate_straight(start, end)
            ))
            rail_map.add_segment(RailSegment(
                id = f"R"
            ))
        # for one block, Setting the Intersection Rail
        
        
        
        
        
        for row in range(4):
            y = -row * (block_h + gap) + block_h / 2 + margin
            x1 = 0 - margin # Start pont of Rail
            x2 = (block_w + gap) * 1 + block_w + margin # End point of Rail

            n1_id = f"N{node_id}"
            n2_id = f"N{node_id+1}"
            rail_map.add_node(RailNode(n1_id, x1, y)) # add node, (Node ID, Position_x, Position_y)
            rail_map.add_node(RailNode(n2_id, x2, y))

            segment = RailSegment(
                id=f"R{segment_id}",
                start_node=n1_id,
                end_node=n2_id,
                points=generate_straight([x1, y], [x2, y]),
                segment_type='straight'
            )
            rail_map.add_segment(segment) # Se
            segment_id += 1
            node_id += 2

        for i in range(3):
            y_base = -i * (block_h + gap) + block_h / 2 + margin
            y_next = -(i+1) * (block_h + gap) + block_h / 2 + margin
            for side in ["left", "right"]:
                if side == "left":
                    x = -margin + curve_r
                    start_angle = -math.pi/2
                    end_angle = 0
                else:
                    x = (block_w + gap)*1 + block_w - curve_r + margin
                    start_angle = math.pi
                    end_angle = math.pi/2

                center = [x, (y_base + y_next)/2]
                pts = generate_turn(center=center, radius=curve_r, angle_start=start_angle, angle_end=end_angle)


                n_start = f"N{node_id}"
                n_end = f"N{node_id+1}"
                rail_map.add_node(RailNode(n_start, pts[0][0], pts[0][1]))
                rail_map.add_node(RailNode(n_end, pts[-1][0], pts[-1][1]))

                seg = RailSegment(
                    id=f"R{segment_id}",
                    start_node=n_start,
                    end_node=n_end,
                    points=pts,
                    segment_type='curve'
                )
                rail_map.add_segment(seg)
                segment_id += 1
                node_id += 2

        return rail_map

    def publish_markers(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "rail"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.0
        marker.color.a = 1.0

        for segment in self.rail_map.segments.values():
            pts = segment.points
            for i in range(len(pts) - 1):
                p1 = Point(x=pts[i][0], y=pts[i][1], z=0.02)
                p2 = Point(x=pts[i+1][0], y=pts[i+1][1], z=0.02)
                marker.points.append(p1)
                marker.points.append(p2)

        self.publisher.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = RailVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()