#!/usr/bin/env python3
"""
    Author : JeongHyeok Lim
    Date : May, 2025
"""
import rclpy
from typing import List, Dict
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from planners.global_path_planner import GlobalPlanner
from UserRequest.msg import UserRequest
"""
    hi-level controller
     input: [pair[user_info, charger_info]]
        charger_info : [ pose ]
        user_info : [ pose, request_time ]
        data : waiting queue -> EV_id - Request Pair

"""
class CentralController(Node):
    """
    High-Level Controller (HLC) for the User-Charger Model OHCS System.
    This class implements the HLC and provides functionality to handle user and charger state
    and control commands in a cooperative systems
    """
    def __init__(self, charger_ids: List[int], t_s:float):
        print(charger_ids, flush = 1)
        self.t_s = t_s
        # Generate the Global Path the vehicles will follow
        # Dijkstra Algorithms.
        """
        user_list : [{user_id, ev_id, location, request_time, priority}]
        charger_list : [{charger_id, location, queue: [user_id, ,...]}]
        pair_list : [[user_id, ev_id, charger_id]]

        """
        self.global_planner = GlobalPlanner() # Find route..
        self.user_list = []
        self.charger_list = []
        self.create_subscription(
            UserRequest,
            '/user_request',
            self.user_callback,
            10
        )
    def user_callback(self, msg: UserRequest):
        user_data = {
            "user_id": msg.user_id,
            "ev_id":msg.ev_id,
            "pose" : (msg.pose.position.x, msg.pose.position.y),
            "request_time": msg.request_time
        }
        self.user_list.append(user_data)


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