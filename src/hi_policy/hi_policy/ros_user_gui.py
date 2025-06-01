#!/usr/bin/env python3
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLineEdit, QLabel
import rclpy
from rclpy.node import Node
from hlc_interfaces.msg import UserRequest
from geometry_msgs.msg import Pose
from builtin_interfaces.msg import Time
import sys

class UserRequestPublisher(Node):
    def __init__(self):
        super().__init__('user_request_gui')
        self.publisher_ = self.create_publisher(UserRequest, '/user_states', 10)

    def publish_request(self, user_id, x, y):
        msg = UserRequest()
        msg.user_id = user_id
        msg.location = Pose()
        msg.location.position.x = float(x)
        msg.location.position.y = float(y)
        msg.location.position.z = 0.0
        now = self.get_clock().now().nanoseconds / 1e9
        msg.request_time = now
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: {msg}")

class App(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle("EV 충전 요청")

        layout = QVBoxLayout()
        self.id_input = QLineEdit()
        self.x_input = QLineEdit()
        self.y_input = QLineEdit()
        layout.addWidget(QLabel("User ID"))
        layout.addWidget(self.id_input)
        layout.addWidget(QLabel("X 위치"))
        layout.addWidget(self.x_input)
        layout.addWidget(QLabel("Y 위치"))
        layout.addWidget(self.y_input)

        send_button = QPushButton("요청 보내기")
        send_button.clicked.connect(self.send_request)
        layout.addWidget(send_button)
        self.setLayout(layout)

    def send_request(self):
        uid = self.id_input.text()
        x = self.x_input.text()
        y = self.y_input.text()
        self.ros_node.publish_request(uid, x, y)

def main():
    app = QApplication(sys.argv)
    rclpy.init()
    ros_node = UserRequestPublisher()

    gui = App(ros_node)
    gui.show()
    app.exec()
    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
