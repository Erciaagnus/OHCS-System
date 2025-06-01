#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from typing import Tuple, Dict, List
from parking_world.rail_visualizer import RailNode, RailSegment, RailMap
from ament_index_python.packages import get_package_share_directory
import json
import os
from collections import defaultdict
import random
from hlc_interfaces.msg import ChargerState, ChargerPath
from geometry_msgs.msg import Pose
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
STATUS = ["idle", "busy"]
# Charger Node
class Charger(Node):
    def __init__(self, charger_id, rail_map, start_node_id):
        super().__init__(f"charger_{charger_id}")
        self.charger_id = charger_id
        self.rail_map = rail_map
        self.location = self.get_node_position(start_node_id) # Get from the Charger Manager/Message
        self.pose = Pose()
        # Get Node position
        self.pose.position.x, self.pose.position.y = self.location
        self.path = []
        self.path_index = 0
        self.status = "idle"

        self.state_pub = self.create_publisher(ChargerState, "/charger_states", 10)
        self.path_sub = self.create_subscription(ChargerPath, "/charger_paths", self.path_callback, 10)
        self.timer = self.create_timer(0.2, self.step)

    def path_callback(self, msg):
        if msg.charger_id != self.charger_id:
            return
        self.path = [(p.position.x, p.position.y) for p in msg.path]
        self.path_index = 0
        self.status = "busy"

    def step(self):
        if self.status == "busy" and self.path_index < len(self.path):
            x, y = self.path[self.path_index]
            self.pose.position.x = x
            self.pose.position.y = y
            self.path_index += 1
            if self.path_index >= len(self.path):
                self.status = "idle"
        self.publish_state() # Update the state at 2D Space

    def publish_state(self):
        msg = ChargerState()
        msg.charger_id = self.charger_id
        msg.location = self.pose
        msg.status = self.status
        self.state_pub.publish(msg)

    def get_node_position(self, node_id):
        node = self.rail_map.nodes[node_id]
        return (node.x, node.y)
