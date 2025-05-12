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

    def add_segment(self, start_id:str, end_id:str, pts, typ = 'straight') -> str:
        """
        -start points, endpoints, type -> self.segments, connected info
        """
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
def generate_turn(center, radius=2.0, angle_start=0, angle_end=math.pi/2, density=20):
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
        self.publisher = self.create_publisher(Marker, 'rail_marker', 10)
        self.timer = self.create_timer(1.0, self.publish_markers)
        self.rail_map = self.build_rail_map()

    def build_rail_map(self):
        rm = RailMap()

        # ─── 파라미터 ───────────────────
        slots_col, slots_row = 6,2   # 슬롯 6×2
        blocks_col,blocks_row = 2,3 # 블록 2열×3행
        slot_w, slot_h = 2.3, 5.1
        gap, margin   = 4.0, 0.5
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
                x0 = bc*(block_w+gap) # block start point.x
                y0 = br*(block_h+gap) # block start point.y
                top_slots_ids = []
                # 상·하단 노드 for one block
                for col in range(slots_col+1): # 0-6
                    x_ctr = x0 + col*slot_w
                    y_ctr = y0 + block_h + margin
                    slot_num = col + 1
                    node_name = f"{zone}{slot_num}"
                    nid = rm.add(x_ctr, y_ctr, name = node_name)
                    top_slots_ids.append(nid)
                
                bottom_slots_ids = []
                for col in range(slots_col + 1): # 0-6
                    x_ctr = x0 + col*slot_w + slot_w/2
                    y_ctr = y0 - slot_h*slots_row - margin
                    slot_num = slots_col + (i+1) # 7~
                    node_name = f"{zone}{slot_num}"
                    nid = rm.add_node(x_ctr, y_ctr, name=node_name)
                    bottom_slots_ids.append(nid)

                # 상·하 직선 for one block # Node name : {zone}{slots_col*slots_row +1, +2(left), +3, +4(right)}
                for i in range(2):
                    rm.add_node(x0-curve_r, y0 + margin - i*curve_r, f"{zone}{slots_col*slots_row + i + 1}")
                    rm.add_node(x0+curve_r+slot_w*slots_col, y0 + margin-i*curve_r, f"{zone}{slots_col*slots_row + i + 3}")
                
        #TODO(2) Define ROAD Node -> Should ADD
        for bc in range(blocks_col):
                for br in range(blocks_row-1):
                    block_idx = br*blocks_col + bc
                    zone = blocks_letters[block_idx]
                    x0 = bc(block_w+gap)
                    y0 = br*(block_h+gap)
                    r1_x = x0 - curve_r
                    r1_y = y0 - slot_h*slots_row - gap / 2
                    r2_x = x0 + curve_r + slot_w*slots_col
                    r2_y = r1_y
                    rm.add_node(r1_x, r1_y, f"{road_zone}{zone}L")
                    rm.add_node(r2_x, r2_y, f"{road_zone}{zone}R")
        
        #TODO(3) Define EDGE in block
        """
            Edge : {start, end, type}
            block id : {zone}{int}
            road id : R{zone}L, R{zone}R
        """
        slots_possible = slots_row*slots_col

        for zone in blocks_letters: # every zone A - F
            # in one blocks
            for i in range(1, slots_col, 1): # 1 ~ 5
                rm.add_segment(f"{zone}{i}", f"{zone}{i+1}", "straight") # 1~6
            for i in range(slots_col+1, slots_possible, 1): # 7 ~ 11
                rm.add_segment(f"{zone}{i}", f"{zone}{i+1}", "straight") # bottom line
            # Top-Left curve/Straight
            rm.add_segment(f"{zone}{slots_possible+1}", f"{zone}{slots_possible+2}", "straight")
            rm.add_segment(f"{zone}{slots_possible+1}", f"{zone}1", "curve")

            # Bottom-Left Curve
            rm.add_segment(f"{zone}{slots_possible+2}", f"{zone}7", "curve")

            # Top-Right curve/Straight
            rm.add_segment(f"{zone}{slots_possible+3}", f"{zone}{slots_possible+4}", "straight")
            rm.add_segment(f"{zone}{slots_possible+3}", f"{zone}{slots_possible+4}", "curve")

            # bottom-Right Curve
            rm.add_segment(f"{zone}{slots_possible+4}", f"{zone}12", "curve")

        #TODO(4) : Creating Edge that connecting the Road Node to Block Node (Intersection)
        """
            Edge : Raod - Block
        """
        for i, zone in enumerate(blocks_letters[:4]):
            next_zone = i+2
            rm.add_segment(f"R{zone}L", f"{zone}{slots_col+1}", "curve")
            rm.add_segment(f"R{zone}L", f"{zone}{slots_possible + 2}", "straight")
            rm.add_segment(f"R{zone}R", f"{zone}{blocks_letters[i+2]}")


        #TODO(5) : Edge Generate Straight / Curve Messaging Publishing
        #TODO(6) : Node Publishing
                rm.add(node)
                for i in range(slots_col):
                    rm.add_seg(top_ids[i],   top_ids[i+1],
                            generate_straight(
                                [rm.nodes[top_ids[i]].x, rm.nodes[top_ids[i]].y],
                                [rm.nodes[top_ids[i+1]].x,rm.nodes[top_ids[i+1]].y]),
                            'straight')
                    rm.add_seg(bot_ids[i],   bot_ids[i+1],
                            generate_straight(
                                [rm.nodes[bot_ids[i]].x, rm.nodes[bot_ids[i]].y],
                                [rm.nodes[bot_ids[i+1]].x,rm.nodes[bot_ids[i+1]].y]),
                            'straight')

                # 좌상 필렛
                c_tl = [ x0-margin+curve_r,   y0+block_h+margin-curve_r ]
                pts_tl = generate_turn(c_tl,curve_r,math.pi/2,math.pi)
                n_tl  = add_node(*pts_tl[0])
                add_seg(top_ids[0],n_tl,pts_tl,'curve')
                # 좌측 직선
                end_tl = pts_tl[-1]; n_et = add_node(*end_tl)
                mid_l  = [x0-margin, y0-margin+curve_r]
                n_ml   = add_node(*mid_l)
                add_seg(n_et,n_ml,generate_straight(end_tl,mid_l),'straight')
                # 좌하 필렛
                c_bl = [ x0-margin+curve_r, y0-margin+curve_r ]
                pts_bl = generate_turn(c_bl,curve_r,math.pi,3*math.pi/2)
                add_seg(n_ml,bot_ids[0],pts_bl,'curve')

                # 우상 필렛
                c_tr = [ x0+block_w+margin-curve_r, y0+block_h+margin-curve_r ]
                pts_tr = generate_turn(c_tr,curve_r,0,math.pi/2)
                n_tr  = add_node(*pts_tr[0])
                add_seg(top_ids[-1],n_tr,pts_tr,'curve')
                # 우측 직선
                end_tr = pts_tr[-1]; n_er = add_node(*end_tr)
                mid_r  = [x0+block_w+margin, y0-margin+curve_r]
                n_mr   = add_node(*mid_r)
                add_seg(n_er,n_mr,generate_straight(end_tr,mid_r),'straight')
                # 우하 필렛
                c_br = [ x0+block_w+margin-curve_r, y0-margin+curve_r ]
                pts_br = generate_turn(c_br,curve_r,3*math.pi/2,2*math.pi)
                add_seg(n_mr,bot_ids[-1],pts_br,'curve')

        return rm
        
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
                marker.points += [p1, p2]

        self.publisher.publish(marker)
        node_m = Marker()
        node_m.header.frame_id = "map"
        node_m.header.stamp = self.get_clock().now().to_msg()
        node_m.ns = "nodes"; node_m.id = 1
        node_m.type = Marker.SPHERE_LIST; node_m.action = Marker.ADD
        node_m.scale.x = 0.2  # 구경 0.2m
        node_m.scale.y = 0.2
        node_m.scale.z = 0.2
        node_m.color.r = 0.0; node_m.color.g = 1.0; node_m.color.b = 0.0; node_m.color.a = 1.0

        for node in self.rail_map.nodes.values():
            node_m.points.append(Point(x=node.x, y=node.y, z=0.05))
        self.publisher.publish(node_m)

        # 3) Node labels
        text_id = 2
        for node in self.rail_map.nodes.values():
            txt = Marker()
            txt.header.frame_id = "map"
            txt.header.stamp = self.get_clock().now().to_msg()
            txt.ns = "labels"; txt.id = text_id
            text_id += 1
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.scale.z = 0.4            # 글자 크기
            txt.color.r = 1.0; txt.color.g = 1.0; txt.color.b = 1.0; txt.color.a = 1.0
            txt.pose.position.x = node.x
            txt.pose.position.y = node.y
            txt.pose.position.z = 0.3
            txt.text = node.id
            self.publisher.publish(txt)
def main(args=None):
    rclpy.init(args=args)
    node = RailVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()