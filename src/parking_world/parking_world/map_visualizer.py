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
            block_spacing_x = 4.8
            block_spacing_y = 3.4
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
