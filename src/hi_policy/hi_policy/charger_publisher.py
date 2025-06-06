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
import threading
from low_policy.charger_node import LowPolicy
import time
"Charger State : {charger_id, location, status, + paired_reqeust_time}"
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

STATUS = ["idle", "moving", "Pre-Charging", "Charging", "Unplugging", "Connector_Retreat"]
# Charger Node
class Charger(Node):
    def __init__(self, charger_id, rail_map, start_node_id):
        super().__init__(f"charger_{charger_id}")
        self.low_policy = LowPolicy(self, rail_map, self.get_logger()) # self = charger
        self.init_done = False
        self.charger_id = charger_id
        self.rail_map = rail_map

        # Start Position
        self.location = self.get_node_position(start_node_id) # Get from the Charger Manager/Message
        self.pose = Pose()
        # Get Node position
        # Pose Data Will Be Updated By Low Level Policy and Publish
        self.pose.position.x, self.pose.position.y = self.location

        self.path = []
        self.path_index = 0
        self.status = "idle"
        self.priority = 1e9 # Default Priority -> Updated by Path Callback message
        self.lock = threading.Lock()

        #Low Policy
        """
        charger.status : # Updated by LowLevel Policy
        charger.path : OK Updated by Path Callback Function
        charger.priority # Updated by Charger Path Callback Function
        charger.current_pos # Why need?
        """
        self.other_chargers: Dict[str, ChargerState] = {} # State
        self.other_priority: Dict[str, float] = {}
        self.other_paths: Dict[str, List[Pose]] = {} # Path
        self.last_transition_time = time.time()

        # Message
        self.state_pub = self.create_publisher(ChargerState, "/charger_states", 10)
        self.path_sub = self.create_subscription(ChargerPath, "/charger_paths", self.path_callback, 10)
        self.timer = self.create_timer(0.2, self.step)
        self.other_state_sub = self.create_subscription(ChargerState, "/charger_states", self.state_callback, 10)

    def state_callback(self, msg:ChargerState):
        if msg.charger_id == self.charger_id:
            return
        # Store Other Charger Info
        self.other_chargers[msg.charger_id] = msg # Charger_id, charger_location[Pose], Charger Status

    def path_callback(self, msg:ChargerPath):
        if msg.charger_id != self.charger_id:
            self.other_paths[msg.charger_id] = msg.path # Store other charger paths
            self.other_priority[msg.charger_id] = round(msg.paired_request_time,2) # Priority : Request Time
            return
        self.path = [(p.position.x, p.position.y) for p in msg.path] # my path
        self.path_index = 0
        if len(self.path) != 0:
            self.status = "moving" # If Get Charger Paths, Update Status
        self.get_logger().info(f"Charger {msg.charger_id} status is {self.status}")
        self.paired_request_time = msg.paired_request_time
        self.priority = round(msg.paired_request_time, 2) # Priority Defined

    def publish_state(self):
        msg = ChargerState()
        msg.charger_id = self.charger_id
        msg.location = self.pose
        msg.status = self.status
        self.state_pub.publish(msg)

    def get_node_position(self, node_id):
        node = self.rail_map.nodes[node_id]
        return (node.x, node.y)

    def step(self):
        charger = self.low_policy.step_charger()
        self.charger_id = charger.charger_id
        self.pose = charger.pose
        self.status = charger.status
        self.publish_state() # Update the state at 2D Space