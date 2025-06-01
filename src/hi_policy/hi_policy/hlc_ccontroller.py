#!/usr/bin/env python3
"""
    Author : JeongHyeok Lim
    Date : May, 2025
"""
import rclpy
from typing import List, Dict, Tuple
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from global_path_planner import GlobalPlanner
from UserRequest import UserRequest
from multiple_agent_path import GlobalPathPlanner
from ament_index_python.packages import get_package_share_directory
import json
import os
from parking_world.rail_visualizer import RailNode, RailSegment, RailMap
from hi_policy.multiple_agent_path import CBSNode, HungarianPair, CBSPlanner, HLCPlanner, UserManager, ChargerManager

"""
    hi-level controller
     input: [pair[user_info, charger_info]]
        charger_info : [ pose ]
        user_info : [ pose, request_time ]
        data : waiting queue -> EV_id - Request Pair
"""

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

class CentralController(Node):
    """
    High-Level Controller (HLC) for the User-Charger Model OHCS System.
    This class implements the HLC and provides functionality to handle user and charger state
    and control commands in a cooperative systems
    """
    def __init__(self, t_s:float):
        self.t_s = t_s
        # Generate the Global Path the vehicles will follow
        """
        user_list : [{user_id, location, request_time, priority}]
        charger_list : [{charger_id, location, queue: [user_id, ,...]}]
        pair_list : [[user_id, ev_id, charger_id]]

        """
        # Map Setting
        self.rail_map = load_rail_map_from_json()
        # Graph Setting
        self.graph = GlobalPlanner(self.rail_map)
        self.global_planner = GlobalPlanner() # Find route..
        self.user_list = [] # Request Info is stored to this list
        self.charger_list = []
        self.create_subscription(
            UserRequest,
            '/user_states',
            self.user_callback,
            10
        )
        self.user_manager = UserManager()
        self.charger_manager = ChargerManager()

## State Machine
    def update_vehicle_states(self,current_time):
        for v_id, v_info in self.vehicle_states.items():
            if v_info['state'] == "WAITING":
                next_node = v_info["path"][v_info["current_index"]]
                reserved = self.reservation_table.get(next_node, [])
                if not any(abs(t-current_time) < self.t_s for t, _ in reserved):
                    self.reservation_table.setdefault(next_node, []).append((current_time, v_id))
                    v_info["state"] = "MOVING"
                else:
                    v_info["state"] = "WAITING"
            elif v_info["state"] == "MOVING":
                v_info["current_index"] += 1
                if v_info["current_index"] >= len(v_info["path"]):
                    v_info["state"] = "CHARGING"
            elif v_info["state"] == "CHARGING":
                if self.check_charging_complete(v_id):
                    v_info["state"] = "COMPLETE"

    def assign_path_to_vehicle(self, vehicle_pair, vehicle_id:str, path: List[str]):
        global_path_planner = GlobalPathPlanner()
        request_time =[vehicle_id, self.user_list["request_time"]]
        vehicle_pairs = vehicle_pair
        global_path_planner(self.graph, vehicle_pairs, request_time)

    def check_charging_complete(self, vehicle_id:str) -> bool:
        #TODO : condition for completing the charging
        return True

    def step_fsm(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        self.update_vehicle_states(now)

    def pair_matching():
        """
        pair with user-charger
        return : [[user_id, ev_id, charger_id]]
        """
        pass

    def get_paths(self, user_info, charger_info):
        """
        Get paths from charger to user
        Args:
            user_info (List): [ id, pose, request_time ]
            charger_info (List): [id, pose, waiting_queue ]
        """
        user_pose = user_info["pose"]
        charger_pose = charger_info["pose"]
        path = self.global_planner.plan(charger_pose, user_pose)
        return path
    def charger_scheduler():
        """
            considering the waiting queue, ETA based dynamic update
        """
    def real_time_monitor():
        """
            moving/arriving, charging, Complete -> state real-time update
        """
    def db_sync():
        """
            Publishing Every state info and store, Provides API
        """
    def API_Layer():
        pass
def main(args=None):
    rclpy.init(args=args)
    node = CentralController(charger_ids=["C01", "C02"], t_s = 10)
    rclpy.spin(node)
    rclpy.shutdown()