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
from planners.global_path_planner import GlobalPlanner
from UserRequest.msg import UserRequest
"""
    hi-level controller
     input: [pair[user_info, charger_info]]
        charger_info : [ pose ]
        user_info : [ pose, request_time ]
        data : waiting queue -> EV_id - Request Pair

"""
STATES = ["IDLE", "WAITING", "MOVING","CHARGING", "COMPLETE"]

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
        ## FSM + Horizon
        self.reservation_table: Dict[str, List[Tuple[float, str]]] = {} #node_id -> List of (reserved_time, vehicle_id)
        self.vehicle_states: Dict[str, Dict] = {}
        self.timer = self.create_timer(self.t_s, self.step_fsm)
    def user_callback(self, msg: UserRequest):
        user_data = {
            "user_id": msg.user_id,
            "ev_id":msg.ev_id,
            "pose" : (msg.pose.position.x, msg.pose.position.y),
            "request_time": msg.request_time
        }
        self.user_list.append(user_data)

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

    def assign_path_to_vehicle(self, vehicle_id:str, path: List[str]):
        self.vehicle_states[vehicle_id] = {
            "state": "WAITING",
            "path" : path, # change to segment_id?
            "current_index":0
        }
        eta = self.get_clock().now().nanoseconds * 1e-9
        safe_margin = 0.5 * self.t_s
        reserved_path = []
        conflict_found = False

        for idx, segment in enumerate(path):
            arrival_time = eta + idx*self.t_s
            reserved_slots = self.reservation_table.get(segment, [])
            for t, _ in reserved_slots:
                if abs(t-arrival_time) < safe_margin:
                    conflict_found = True
                    print(f"[RESERVED CONFLICT] {vehicle_id} at {segment} time {arrival_time:.2f}")
                    break
            if conflict_found:
                break
            else:
                reserved_path.append((segment, arrival_time))
        if conflict_found:
            print(f"[RESERVE FAIL] Vehicle {vehicle_id} could not reserve path.")
            # [Option A] 대기 시간 삽입 or 경로 재계산을 여기에 삽입 가능
            return  # 일단 배정 중단
        # 예약 등록 (충돌 없을 때만)
        for segment, t in reserved_path:
            self.reservation_table.setdefault(segment, []).append((t, vehicle_id))

        # FSM 등록
        self.vehicle_states[vehicle_id] = {
            "state": "WAITING",
            "path": path,
            "current_index": 0
        }
        print(f"[RESERVE SUCCESS] Vehicle {vehicle_id} assigned path.")

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