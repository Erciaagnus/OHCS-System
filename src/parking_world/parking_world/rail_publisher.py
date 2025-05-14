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
from typing import Optional
from visualization_msgs.msg import MarkerArray
import os
class RailNode():
    def __init__(self, id, x, y):
        self.id = id # ex. "A1", "N1" 처럼 고유 식별자
        self.x = x  # 실제 맵 상 x 좌표
        self.y = y # 실제 맵 상 y 좌표
        self.connected_segments = [] # 이 노드에서 출발하는 segment ID 목록

class RailSegment:
    def __init__(self, id, start_node, end_node, points = None, segment_type = 'straight'):
        self.id = id
        self.start_node = start_node
        self.end_node = end_node
        self.points = points or []
        self.segment_type = segment_type
class RailMap:
    def __init__(self):
        self.nodes = {} # id-> RailNode
        self.segments = {} # id->RailSegment
        self._coord2id = {} #(x,y) -> id
        self._next_node_idx = 1
        self.radius = 1.2
    def add_node(self, x:float, y:float, name: str = None )->str:
        """
        - 소수점 셋째 자리까지 반올림한 (x,y) 좌표를 키로 중복 방지
        - 새 노드가 필요하면 N1, N2… 순차 ID 발급
        - RailNode를 생성·등록하고, 노드 ID를 반환
        """
        key = (round(x,3), round(y,3))
        if key not in self._coord2id:
            if name:
                nid = name
            else:
                nid = f"N{self._next_node_idx}"
                self._next_node_idx += 1
            self._coord2id[key] = nid
            self.nodes[nid] = RailNode(nid, x, y)
        return self._coord2id[key]

    def add_segment(self, start_id:str, end_id:str, typ = 'straight', dir:str = None, pts : Optional[list]=None) -> str:
        """
        -start points, endpoints, type -> self.segments, connected info
        """
        if pts is None:
            start_xy = [self.nodes[start_id].x, self.nodes[start_id].y]
            end_xy = [self.nodes[end_id].x, self.nodes[end_id].y]
            if typ == 'straight':
                pts = generate_straight(start_xy, end_xy)
            elif typ == 'curve':
                dx = end_xy[0] - start_xy[0]
                dy = end_xy[1] - start_xy[1]
                if dir == "up":
                    if dx > 0 and dy > 0:
                        center = [end_xy[0], start_xy[1]]
                        start_angle = math.pi / 2
                        end_angle = math.pi
                    elif dx > 0 and dy < 0:
                        center = [start_xy[0], end_xy[1]]
                        start_angle = 0
                        end_angle = math.pi / 2
                    elif dx < 0 and dy > 0:
                        center = [end_xy[0], start_xy[1]]
                        start_angle = 0
                        end_angle = math.pi / 2
                    elif dx < 0 and dy < 0:
                        center = [start_xy[0], end_xy[1]]
                        start_angle = math.pi / 2
                        end_angle = math.pi
                elif dir == "down":
                    if dx > 0 and dy > 0:
                        center = [start_xy[0], end_xy[1]]
                        start_angle = math.pi * 3 / 2
                        end_angle = math.pi * 2
                    elif dx > 0 and dy < 0:
                        center = [end_xy[0], start_xy[1]]
                        start_angle = math.pi
                        end_angle = math.pi * 3 / 2
                    elif dx < 0 and dy > 0:
                        center = [start_xy[0], end_xy[1]]
                        start_angle = math.pi
                        end_angle = math.pi * 3/2
                    elif dx < 0 and dy < 0:
                        center = [end_xy[0], start_xy[1]]
                        start_angle = math.pi * 3 / 2
                        end_angle = math.pi * 2
                else:
                    raise ValueError("Curve direction 'dir' must be either 'up' or 'down'")

                pts = generate_turn(center, angle_start=start_angle, angle_end=end_angle)
        if typ == 'curve':
            start_xy = [self.nodes[start_id].x, self.nodes[start_id].y]
            end_xy = [self.nodes[end_id].x, self.nodes[end_id].y]
            pts = [start_xy] + pts + [end_xy]
        rid = f"E{len(self.segments)+1}"
        seg = RailSegment(rid, start_id, end_id, pts, typ)
        self.segments[rid] = seg
        self.nodes[start_id].connected_segments.append(rid)
        self.nodes[end_id].connected_segments.append(rid)
        return rid
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
    def save_to_file(self,path):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, 'w') as f:
            f.write(self.to_json())

def generate_straight(start, end, density=10):
    if start is None or end is None:
        raise ValueError("start 또는 end 좌표가 None입니다.")

    start = np.array(start)
    end = np.array(end)
    dist = np.linalg.norm(end - start)
    count = max(2, int(dist * density))
    return [list(p) for p in np.linspace(start, end, count)]

    # Create Turning Points
def generate_turn(center, radius=1.2, angle_start=0, angle_end=math.pi/2, density=20):
    if center is None or not isinstance(center, (list, tuple)):
        raise ValueError("center는 [x, y] 형태의 리스트나 튜플이어야 합니다.")
    return [
        [center[0] + radius * math.cos(theta),
         center[1] + radius * math.sin(theta)]
        for theta in np.linspace(angle_start, angle_end, density)
    ]


class RailVisualizer(Node):
    def __init__(self):
        super().__init__('rail_visualizer')
        self.publisher = self.create_publisher(MarkerArray, 'rail_marker_array', 10)
        self.timer = self.create_timer(1.0, self.publish_markers)
        self.rail_map = self.build_rail_map()
        self.markers_published = False

    def build_rail_map(self):
        rm = RailMap()

        # ─── 파라미터 ───────────────────
        slots_col, slots_row = 6,2   # 슬롯 6×2
        blocks_col,blocks_row = 2,3 # 블록 2열×3행
        slot_w, slot_h = 2.3, 5.1
        main_road_w = 4.8
        gap, margin   = 3.4, 0.5
        curve_r       = 1.2

        block_w = slots_col*slot_w    # 13.8
        block_h = slots_row*slot_h    # 10.2
        blocks_letters = ['A', 'B', 'C', 'D', 'E', 'F']
        road_zone = "R"
        #TODO(1) Define block node
        # ─── 1) 블록 외곽: 상·하선 ─────────
        for br in range(blocks_row): # creating the nodes for every block #0,1,2
            for bc in range(blocks_col): # 0, 1
                block_idx = br*blocks_col + bc
                zone = blocks_letters[block_idx]
                x0 = bc*(block_w+main_road_w) # block start point.x
                y0 = -br*(block_h+gap) # block start point.y
                top_slots_ids = []
                # 상·하단 노드 for one block
                # Top Node
                for col in range(slots_col+1): # 0-6 6slots, 7node
                    x_ctr = x0 + col*slot_w
                    y_ctr = y0 + margin
                    slot_num = col + 1
                    node_name = f"{zone}{slot_num}"
                    nid = rm.add_node(x_ctr, y_ctr, name = node_name)
                    top_slots_ids.append(nid)
                # Bottom Node
                bottom_slots_ids = []
                for col in range(slots_col+1): # 0-6
                    x_ctr = x0 + col*slot_w
                    y_ctr = y0 - slot_h*slots_row - margin
                    slot_num = slots_col + (col+2) # 8~14
                    node_name = f"{zone}{slot_num}"
                    nid = rm.add_node(x_ctr, y_ctr, name=node_name)
                    bottom_slots_ids.append(nid)

                # 상·하 직선 for one block # Node name : {zone}{slots_col*slots_row +1, +2(left), +3, +4(right)}

                rm.add_node(x0-curve_r, y0 + margin - curve_r, f"{zone}{slots_col*slots_row +  3}")
                rm.add_node(x0-curve_r, y0 - slot_h*2 - margin + curve_r, f"{zone}{slots_col*slots_row +  4}")
                rm.add_node(x0+curve_r+slot_w*slots_col, y0 + margin-curve_r, f"{zone}{slots_col*slots_row + 5}")
                rm.add_node(x0+curve_r+slot_w*slots_col, y0 - slot_h*2 - margin + curve_r, f"{zone}{slots_col*slots_row + 6}")


        #TODO(2) Define ROAD Node -> Should ADD
        for bc in range(blocks_col):
                for br in range(blocks_row-1):
                    block_idx = br*blocks_col + bc
                    zone = blocks_letters[block_idx]
                    x0 = bc*(block_w+main_road_w)
                    y0 = -br*(block_h+gap)
                    r1_x = x0 - curve_r
                    r1_y = y0 - slot_h*slots_row - gap / 2
                    r2_x = x0 + curve_r + slot_w*slots_col
                    r2_y = r1_y
                # Between row
                    rm.add_node(r1_x, r1_y, f"R{zone}{blocks_letters[block_idx+2]}L")
                    rm.add_node(r2_x, r2_y, f"R{zone}{blocks_letters[block_idx+2]}R")
        for bc in range(blocks_col-1): # 0
            for br in range(blocks_row): # 0, 1, 2
                # Between col
                block_idx = br*blocks_col + bc
                zone = blocks_letters[block_idx]
                x0 = bc*(block_w+main_road_w)
                y0 = -br*(block_h+gap)
                r3_x = x0 +slot_w*slots_col+ main_road_w/2 # Top
                r3_y = y0 + margin
                r4_x = x0 + slot_w*slots_col + main_road_w/2
                r4_y = y0 - slot_h*slots_row - margin
                rm.add_node(r3_x, r3_y, f"R{zone}{blocks_letters[block_idx+1]}T")
                rm.add_node(r4_x, r4_y, f"R{zone}{blocks_letters[block_idx+1]}B")


        #TODO(3) Define EDGE in block
        """
            Edge : {start, end, type}
            block id : {zone}{zone}{int}
            road id : R{zone}L, R{zone}R
        """
        slots_possible = slots_row*slots_col

        for zone in blocks_letters: # every zone A - F
            # in one blocks
            for i in range(1, slots_col+1, 1): # 1 ~ 6
                rm.add_segment(f"{zone}{i}", f"{zone}{i+1}", "straight") # 1~7
            for i in range(slots_col+2, slots_possible+2, 1): # 8~13
                rm.add_segment(f"{zone}{i}", f"{zone}{i+1}", "straight") # bottom line 8~14
            # Top-Left curve/Straight
            rm.add_segment(f"{zone}15", f"{zone}16", "straight")
            rm.add_segment(f"{zone}15", f"{zone}1", "curve", "up")
            rm.add_segment(f"{zone}16", f"{zone}8", "curve", "down")
            rm.add_segment(f"{zone}14", f"{zone}18", "curve", "down")
            rm.add_segment(f"{zone}17", f"{zone}7", "curve", "up")
            rm.add_segment(f"{zone}17", f"{zone}18", "straight")

            '''
                    slots_col, slots_row = 6,2   # 슬롯 6×2
                    blocks_col,blocks_row = 2,3 # 블록 2열×3행
                    slot_w, slot_h = 2.3, 5.1
                    main_road_w = 4.8
                    gap, margin   = 3.4, 0.5
                    curve_r       = 1.2
            '''
            # #15
            # rm.add_segment(f"{zone}{slots_row*slots_col+3}", f"{zone}{slots_row*slots_col+4}", "straight")
            # rm.add_segment(f"{zone}{slots_row*slots_col+3}", f"{zone}1", "curve", "up")

            # # Bottom-Left Curve # 16
            # rm.add_segment(f"{zone}{slots_row*slots_col+4}", f"{zone}{slots_row*slots_col + 2}", "curve", "down")

            # # Top-Right curve/Straight #17
            # rm.add_segment(f"{zone}{slots_row*slots_col+5}", f"{zone}{slots_col*slots_row+6}", "straight")
            # rm.add_segment(f"{zone}{slots_row*slots_col+5}", f"{zone}{slots_col+1}", "curve", "up")

            # # bottom-Right Curve # 14
            # rm.add_segment(f"{zone}{slots_row*slots_col+2}", f"{zone}{slots_col*slots_row + 6}", "curve", "down")

        #TODO(4) : Creating Edge that connecting the Road Node to Block Node (Intersection)
        """
            Edge : Raod - Block
        """
        for row in range(blocks_row): # 0, 1, 2
            zone_num = row*blocks_col # 0, 2, 4
            zone = blocks_letters[zone_num] # A, C, E
            # Top Rail with zone straight, curve
            rm.add_segment(f"R{zone}{blocks_letters[zone_num+1]}T", f"{zone}{slots_col*slots_row+5}", "curve", "up")
            rm.add_segment(f"R{zone}{blocks_letters[zone_num+1]}T", f"{zone}{slots_col+1}", "straight")
            #rm.add_segment(f"R{zone}{blocks_letters[zone_num+1]}T", f"{blocks_letters[zone_num + 1]}15", "curve", "up") # Problem?
            rm.add_segment(f"R{zone}{blocks_letters[zone_num+1]}T", f"{blocks_letters[zone_num + 1]}{slots_col*slots_row+3}", "curve", "up") # Problem?
            rm.add_segment(f"R{zone}{blocks_letters[zone_num+1]}T", f"{blocks_letters[zone_num + 1]}1", "straight")


            # Bottom Rail with zone straight, curve
            rm.add_segment(f"R{zone}{blocks_letters[zone_num+1]}B", f"{zone}{slots_col*slots_row+6}", "curve", "down") # Bottom Road -
            rm.add_segment(f"R{zone}{blocks_letters[zone_num+1]}B", f"{zone}{slots_col*slots_row +2}", "straight")
            rm.add_segment(f"R{zone}{blocks_letters[zone_num+1]}B", f"{blocks_letters[zone_num+1]}{slots_col*slots_row+4}", "curve", "down")
            rm.add_segment(f"R{zone}{blocks_letters[zone_num+1]}B", f"{blocks_letters[zone_num+1]}{slots_col+2}", "straight")

        for col in range(blocks_col): # Every Col 0, 1
            for row in range(blocks_row-1): # 0, 1
                    zone_num = col+row*blocks_col # 0, 1, 2, 3
                    zone = blocks_letters[zone_num] # Current zone # A B C D
                    if zone_num % blocks_col == 0: # 0, 2 : A C
                        # Left Rail Road : with Left - Curve Top Block
                        rm.add_segment(f"R{zone}{blocks_letters[zone_num+blocks_col]}L", f"{zone}{slots_col+2}", "curve", "up") # Road Left - 7 segment
                        # Left Rail Road : with Left - Straight Top Block
                        rm.add_segment(f"R{zone}{blocks_letters[zone_num+blocks_col]}L", f"{zone}{slots_col*slots_row+4}", "straight") # Road Left - 14 segment
                        # Left Rail Road : with Left - Straight : Bottom Block
                        rm.add_segment(f"R{zone}{blocks_letters[zone_num+blocks_col]}L", f"{blocks_letters[zone_num+blocks_col]}{slots_col*slots_row+3}", "straight") # Road Left - 13 segment
                        # Left Rail Road : with Left - Curve : Bottom Block
                        rm.add_segment(f"R{zone}{blocks_letters[zone_num+blocks_col]}L", f"{blocks_letters[zone_num+blocks_col]}1", "curve", "down") # Road Left - 1 segment

                        # Right Rail Load with - Straight Top Block
                        rm.add_segment(f"R{zone}{blocks_letters[zone_num+blocks_col]}R", f"{zone}{slots_col*slots_row+6}", "straight") # Road Left - 16 segment
                        # with Bottom - Curve Top Block:Road
                        rm.add_segment(f"R{zone}{blocks_letters[zone_num+blocks_col]}R", f"R{zone}{blocks_letters[zone_num+1]}B", "curve", "up") # Road Left - 7 segment
                        # Right Rail Road with Straight - Bottom block
                        rm.add_segment(f"R{zone}{blocks_letters[zone_num+blocks_col]}R", f"{blocks_letters[zone_num+blocks_col]}{slots_col*slots_row+5}", "straight") # Road Right - Road Bottom of zone segment
                        # Right Rial Road with Curve - Bottom Block
                        rm.add_segment(f"R{zone}{blocks_letters[zone_num+blocks_col]}R", f"R{blocks_letters[zone_num+blocks_col]}{blocks_letters[zone_num+blocks_col + 1]}T", "curve", "down") # Road Right - Road bottom of next zone segment
                        rm.add_segment(f"R{zone}{blocks_letters[zone_num+blocks_col]}R", f"{zone}{slots_col*slots_row+2}", "curve", "up") # Road Right - Road bottom of next zone segment
                        rm.add_segment(f"R{zone}{blocks_letters[zone_num+blocks_col]}R", f"{blocks_letters[zone_num + blocks_col]}{slots_col+1}", "curve", "down") # Road Right - Road bottom of next zone segment
                    elif zone_num % blocks_col == blocks_col-1: # 1, 3 # B, D
                        rm.add_segment(f"R{zone}{blocks_letters[zone_num+blocks_col]}R", f"{zone}{slots_col*slots_row+2}", "curve", "up") # Road Right - 14 segment
                        # Left Rail Road : with Left - Straight Top Block
                        rm.add_segment(f"R{zone}{blocks_letters[zone_num+blocks_col]}R", f"{zone}{slots_col*slots_row+6}", "straight") # Road Left - 14 segment
                        # Left Rail Road : with Left - Straight : Bottom Block
                        rm.add_segment(f"R{zone}{blocks_letters[zone_num+blocks_col]}R", f"{blocks_letters[zone_num+blocks_col]}{slots_col*slots_row+5}", "straight") # Road Left - 13 segment
                        # Left Rail Road : with Left - Curve : Bottom Block
                        rm.add_segment(f"R{zone}{blocks_letters[zone_num+blocks_col]}R", f"{blocks_letters[zone_num+blocks_col]}{slots_col+1}", "curve", "down") # Road Left - 1 segment

                        # Right Rail Load with - Straight Top Block
                        rm.add_segment(f"R{zone}{blocks_letters[zone_num+blocks_col]}L", f"{zone}{slots_col*slots_row+4}", "straight") # Road Left - 14 segment
                        # with Bottom - Curve Top Block:Road
                        rm.add_segment(f"R{zone}{blocks_letters[zone_num+blocks_col]}L", f"R{blocks_letters[zone_num-1]}{blocks_letters[zone_num]}B", "curve", "up") # Road Left - 7 segment #RBCB and RCBB == same thing
                        # Right Rail Road with Straight - Bottom block
                        rm.add_segment(f"R{zone}{blocks_letters[zone_num+blocks_col]}L", f"{blocks_letters[zone_num+blocks_col]}{slots_col*slots_row+3}", "straight") # Road Right - Road Bottom of zone segment
                        # Right Rial Road with Curve - Bottom Block
                        rm.add_segment(f"R{zone}{blocks_letters[zone_num+blocks_col]}L", f"R{blocks_letters[zone_num+blocks_col-1]}{blocks_letters[zone_num+blocks_col]}T", "curve", "down") # Road Right - Road bottom of next zone segment
                        rm.add_segment(f"R{zone}{blocks_letters[zone_num+blocks_col]}L", f"{zone}{slots_col+2}", "curve", "up") # Road Right - Road bottom of next zone segment
                        rm.add_segment(f"R{zone}{blocks_letters[zone_num+blocks_col]}L", f"{blocks_letters[zone_num + blocks_col]}1", "curve", "down") # Road Right - Road bottom of next zone segment
                    else:
                        print("Please Check the column")
            #             rm.add_segment(f"R{zone}{blocks_letters[zone_num+blocks_col]}L", f"{zone}{slots_col*slots_row+4}", "straight") # Road Left - 14 segment
            #             # with Bottom - Curve Top Block:Road
            #             rm.add_segment(f"R{zone}{blocks_letters[zone_num+blocks_col]}L", f"R{blocks_letters[zone_num-blocks_col-1]}{zone_num-blocks_col}B", "curve", "up") # Road Left - 7 segment #RBCB and RCBB == same thing
            #             # Right Rail Road with Straight - Bottom block
            #             rm.add_segment(f"R{zone}{blocks_letters[zone_num+blocks_col]}L", f"{blocks_letters[zone_num+blocks_col]}{slots_col*slots_row+3}", "straight") # Road Right - Road Bottom of zone segment
            #             # Right Rial Road with Curve - Bottom Block
            #             rm.add_segment(f"R{zone}{blocks_letters[zone_num+blocks_col]}L", f"R{blocks_letters[zone_num-1]}{blocks_letters[zone_num]}T", "curve", "down") # Road Right - Road bottom of next zone segment
            #             rm.add_segment(f"R{zone}{blocks_letters[zone_num+blocks_col]}L", f"{zone}{slots_col+2}", "curve", "up") # Road Right - Road bottom of next zone segment
            #             rm.add_segment(f"R{zone}{blocks_letters[zone_num+blocks_col]}L", f"{blocks_letters[zone_num + blocks_col]}1", "curve", "down")
            #                                 # Right Rail Load with - Straight Top Block
            #             # Right Rail Load with - Straight Top Block
            #             rm.add_segment(f"R{zone}{blocks_letters[zone_num+blocks_col]}R", f"{zone}{slots_col*slots_row+6}", "straight") # Road Left - 16 segment
            #             # with Bottom - Curve Top Block:Road
            #             rm.add_segment(f"R{zone}{blocks_letters[zone_num+blocks_col]}R", f"R{zone}{blocks_letters[zone_num+1]}B", "curve", "up") # Road Left - 7 segment
            #             # Right Rail Road with Straight - Bottom block
            #             rm.add_segment(f"R{zone}{blocks_letters[zone_num+blocks_col]}R", f"{blocks_letters[zone_num+blocks_col]}{slots_col*slots_row+5}", "straight") # Road Right - Road Bottom of zone segment
            #             # Right Rial Road with Curve - Bottom Block
            #             rm.add_segment(f"R{zone}{blocks_letters[zone_num+blocks_col]}R", f"R{blocks_letters[zone_num+blocks_col]}{blocks_letters[zone_num+blocks_col + 1]}T", "curve", "down") # Road Right - Road bottom of next zone segment
            #             rm.add_segment(f"R{zone}{blocks_letters[zone_num+blocks_col]}R", f"{zone}{slots_col*slots_row+2}", "curve", "up") # Road Right - Road bottom of next zone segment
            #             rm.add_segment(f"R{zone}{blocks_letters[zone_num+blocks_col]}R", f"{blocks_letters[zone_num + blocks_col]}{slots_col+1}", "curve", "down") # Road Right - Road bottom of next zone segment


        else:
            print("The column number of blocks is one. Please Set Again")
        #TODO(5) : Edge Generate Straight / Curve Messaging Publishing
        #TODO(6) : Node Publishing

        # print("\n[디버깅: F18 → E17 또는 유사 연결]")
        # for sid, seg in rm.segments.items():
        #     s, e = seg.start_node, seg.end_node
        #     if (s.endswith("18") and e.endswith("17")) or (s.endswith("17") and e.endswith("18")):
        #         print(f"  {sid}: {s} → {e} ({seg.segment_type})")
        #         for i, pt in enumerate(seg.points):
        #             print(f"    pt[{i}] = {pt}")

        return rm

    def publish_markers(self):
        if self.markers_published:
            return

        marker_array = MarkerArray()

        # --- 1. 세그먼트 선분 (LINE_LIST)
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
                rail_line.points.append(Point(x=segment.points[i][0], y=segment.points[i][1], z=0.02))
                rail_line.points.append(Point(x=segment.points[i+1][0], y=segment.points[i+1][1], z=0.02))
        marker_array.markers.append(rail_line)

        # --- 2. 텍스트 마커
        text_id = 1000
        for segment in self.rail_map.segments.values():
            for nid in [segment.start_node, segment.end_node]:
                node = self.rail_map.nodes[nid]
                txt = Marker()
                txt.header.frame_id = "map"
                txt.header.stamp = self.get_clock().now().to_msg()
                txt.ns = "edge_labels"
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

        # 한 번에 퍼블리시
        self.publisher.publish(marker_array)
def save_map_to_file():
    rail_map = RailVisualizer().build_rail_map()
    rail_map.save_to_file('rail_map.json')
def main(args=None):
    rclpy.init(args=args)
    node = RailVisualizer()
    rail_map = node.build_rail_map()
    rail_map.save_to_file(os.path.join(os.path.dirname(__file__), 'maps', 'rail_map.json'))    
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()