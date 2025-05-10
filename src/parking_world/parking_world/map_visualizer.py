#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Duration
import math

class ParkingLotVisualizer(Node):
    def __init__(self):
        super().__init__('map_visualizer')
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        self.get_logger().info("ParkingLotVisualizer initialized.")
        print("== Node Start ==")

        # 플래그로 중복 방지
        self.once_timer_started = False

        # 초기화 후 0.1초 뒤에 주기 타이머 설정
        self.create_timer(0.1, self.init_timer)

    def init_timer(self):
        if self.once_timer_started:
            return
        self.once_timer_started = True
        self.get_logger().info("Starting periodic marker publishing timer.")
        self.create_timer(1.0, self.publish_markers)
    def add_arc(self, rail_marker, center_x, center_y, radius, start_deg, end_deg, clockwise=True, step_deg=10):
        step = -step_deg if clockwise else step_deg
        angle_range = range(start_deg, end_deg + step, step)
        points = []
        for deg in angle_range:
            rad = math.radians(deg)
            x = center_x + radius * math.cos(rad)
            y = center_y + radius * math.sin(rad)
            points.append(Point(x=x, y=y, z=0.05))
        for i in range(len(points) - 1):
            rail_marker.points.append(points[i])
            rail_marker.points.append(points[i + 1])
    def publish_markers(self):
        try:
            spot_w, spot_h = 2.3, 5.1
            block_spacing_x = 2.0
            block_spacing_y = 2.0
            marker_id = 0

            # 주차칸 선(Line List) 마커 생성
            line_marker = Marker()
            line_marker.header.frame_id = "map"
            line_marker.header.stamp = self.get_clock().now().to_msg()
            line_marker.ns = "parking_lines"
            line_marker.id = marker_id
            line_marker.type = Marker.LINE_LIST
            line_marker.action = Marker.ADD
            line_marker.scale.x = 0.05  # 선 두께
            line_marker.color.r = 1.0
            line_marker.color.g = 1.0
            line_marker.color.b = 1.0
            line_marker.color.a = 1.0
            line_marker.lifetime = Duration(sec=0)

            # 3행 × 2열 주차장 블록
            for block_row in range(3):
                for block_col in range(2):
                    base_x = block_col * (spot_w * 6 + block_spacing_x)
                    base_y = -block_row * (spot_h * 2 + block_spacing_y)

                    for i in range(6):  # 가로 6칸
                        for j in range(2):  # 세로 2줄
                            x = base_x + i * (spot_w + 0.1)
                            y = base_y - j * (spot_h + 0.1)

                            corners = [
                                (x, y),
                                (x + spot_w, y),
                                (x + spot_w, y - spot_h),
                                (x, y - spot_h)
                            ]

                            for k in range(4):
                                p1 = Point(x=corners[k][0], y=corners[k][1], z=0.02)
                                p2 = Point(x=corners[(k + 1) % 4][0], y=corners[(k + 1) % 4][1], z=0.02)
                                line_marker.points.append(p1)
                                line_marker.points.append(p2)

            self.marker_pub.publish(line_marker)
            self.get_logger().info("Published parking lot lines as LINE_LIST.")

            # === OHT 레일 + 곡선 교차로 ===
            rail_marker = Marker()
            rail_marker.header.frame_id = "map"
            rail_marker.header.stamp = self.get_clock().now().to_msg()
            rail_marker.ns = "oht_rails"
            rail_marker.id = 9999
            rail_marker.type = Marker.LINE_LIST
            rail_marker.action = Marker.ADD
            rail_marker.scale.x = 0.1
            rail_marker.color.r = 1.0
            rail_marker.color.g = 0.5
            rail_marker.color.b = 0.0
            rail_marker.color.a = 1.0
            rail_marker.lifetime = Duration(sec=0)

            radius = 2.0

            for row in range(3):
                for col in range(2):
                    base_x = col * (spot_w * 6 + block_spacing_x) # two blocks in column
                    print("base_x", base_x)
                    base_y = -row * (spot_h * 2 + block_spacing_y)
                    width = 6 * (spot_w + 0.1)
                    height = 2 * (spot_h + 0.1)

                    # 블록 외곽 감싸는 레일
                    margin = 0.3
                    corners = [
                        (base_x - margin, base_y + margin),
                        (base_x + width + margin, base_y + margin),
                        (base_x + width + margin, base_y - height - margin),
                        (base_x - margin, base_y - height - margin)
                    ]
                    for i in range(4):
                        self.add_line(rail_marker,
                                      corners[i][0], corners[i][1],
                                      corners[(i + 1) % 4][0], corners[(i + 1) % 4][1])

                    # 곡선 위치 계산 (블록의 실제 모서리 바깥쪽)
                    arc_offset = 0.3 + radius
                    arcs = [
                        (base_x - arc_offset, base_y + arc_offset, 180, 270, True),   # 좌상
                        (base_x + width + arc_offset, base_y + arc_offset, 270, 360, True),  # 우상
                        (base_x + width + arc_offset, base_y - height - arc_offset, 0, 90, True),  # 우하
                        (base_x - arc_offset, base_y - height - arc_offset, 90, 180, True)   # 좌하
                    ]
                    for cx, cy, start, end, cw in arcs:
                        self.add_arc(rail_marker, cx, cy, radius, start, end, clockwise=cw)

            # === 공통 수직 레일 (중앙 x값 기준 1줄)
            center_x = spot_w*6 + block_spacing_x / 2
            print("center_x",center_x)
            print("block_spacing_x", block_spacing_x)
            y_top = 0.0
            y_bottom = -3 * (spot_h * 2 + block_spacing_y)
            self.add_line(rail_marker, center_x, y_top, center_x, y_bottom)

            self.marker_pub.publish(rail_marker)
        except Exception as e:
            self.get_logger().error(f"Failed to publish markers: {e}")

    def add_line(self, marker, x1, y1, x2, y2):
        p1 = Point(x=x1, y=y1, z=0.05)
        p2 = Point(x=x2, y=y2, z=0.05)
        marker.points.append(p1)
        marker.points.append(p2)


def main(args=None):
    rclpy.init(args=args)
    node = ParkingLotVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
