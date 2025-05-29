#!/usr/bin/env python3
import json
import os
import heapq
import numpy as np
from typing import List, Tuple, Dict
from collections import defaultdict
from parking_world.rail_visualizer import RailNode, RailSegment, RailMap
from ament_index_python.packages import get_package_share_directory
from scipy.optimize import linear_sum_assignment
from rclpy.node import Node
import rclpy
from hlc_interfaces.msg import ChargerState, UserRequest
"""
    Message Formation
     User Message : {user_id, ev_id, location, request_time}
     Charger_lsit : {charger_id, location, queue: [user_id, ,,,]}
     Graph Info : Node, Edge
     Rail Node ' s Method
        1. Node - [ id, x, y ]
        2. Segments ( Edge ) - [ id, start_node, end_node, points, type ]

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


    def get_idle_chargers(self) -> List[Dict]:
        return [
            {"charger_id": cid, "location:": info["location"]}
            for cid, info in self.charger_states.items()
            if info["status"] == "idle"
        ]

class Constraint:
    def __init__(self, agent: str, time: int, node: str):
        self.agent = agent
        self.time = time
        self.node = node
    def conflicts(self, agent:str, time:int, node:str) -> bool:
        return self.agent != agent and self.time == time and self.node == node

class CBSNode:
    def __init__(self, paths : Dict[str, List[str]], constraints: List[Constraint], cost: int):
        self.paths = paths
        self.constraints = constraints
        self.cost = cost
    def __lt__(self, other):
        return self.cost < other.cost

class HungarianPair:
    def __init__(self, user_list : List[Dict], charger_list: List[Dict]):
        self.user_list = user_list
        self.charger_list = charger_list

    #TODO(1) : Pair Functions
    """
    Args:
        User_list -> [{"user_id":  , "location":  , "requeust_time":   }, {      }]
        Charger_list -> [{"charger_id":  , "location":  , "state" : }, {      }]

    Return:
        pairs = [ ('user_id', 'charger_id' ), ( 'user_id', 'charger_id'), (  )], charge_queus = ['user_id']
    """
    def user_ev_pair(self, dist_wgt:float=1.0, time_wgt:float = 0.1) -> List[Tuple[str, str]]:
        def euclidean(a,b):
            return ((a[0]-b[0])**2 + (a[1]-b[1])**2)**0.5
        
        num_users = len(self.user_list)
        num_chargers = len(self.charger_list)
        N = max(num_users, num_chargers)

        # Array of user and charger
        cost_matrix = np.full((N, N), fill_value=1e6)
        for i, user in enumerate(self.user_list):
            for j, charger in enumerate(self.charger_list):
                dist = euclidean(user['location'], charger['location'])
                t = user['request_time']
                cost_matrix[i,j] = dist_wgt*dist + time_wgt*t
        row_idx, col_idx = linear_sum_assignment(cost_matrix)
        pairs = []
        unmatched_users =set(range(num_users))
        for i, j in zip(row_idx, col_idx):
            if i < num_users and j < num_chargers:
                pairs.append((self.user_list[i]['user_id'], self.charger_list[j]['charger_id']))
                unmatched_users.discard(i)
        remaining_users = [self.user_list[i] for i in unmatched_users]
        charger_goals = [pair[1] for pair in pairs]
        charger_goal_map = {cid: loc for cid, loc in zip(
            charger_goals,
            [ch['location'] for ch in self.charger_list if ch['charger_id'] in charger_goals]
        )}
        charger_queues = defaultdict(list)
        for user in remaining_users:
            closest_cid = min(charger_goal_map, key=lambda c: euclidean(user['location'], charger_goal_map[c]))
            charger_queues[closest_cid].append(user['user_id'])
        return pairs, charger_queues

class CBSPlanner:
    """
        Return:
        pairs = [ ('user_id', 'charger_id' ), ( 'user_id', 'charger_id')], charge_queus = ['user_id']
    """
    def __init__(self, rail_map:RailMap, vehicle_pairs,  user_info, charger_info):
        self.rail_map = rail_map
        self.graph = self.build_graph(rail_map)
        self.vehicle_ids = [f'veh{i}' for i in range(len(vehicle_pairs))]
        self.vehicle_pairs = dict(zip(self.vehicle_ids, vehicle_pairs))
        self.user = {u['user_id']: u for u in user_info}
        self.charger = {c['charger_id']: c for c in charger_info}
        # Position -> Node for Vehicle Pairs
        for vid, (uid, cid) in zip(self.vehicle_ids, vehicle_pairs):
            start_node = self.get_closest_node(self.user[uid]['location'])
            goal_node = self.get_closest_node(self.charger[cid]['location'])
            self.vehicle_pairs[vid] = (start_node, goal_node)
    
    # UTILS
    def compute_cost(self, paths: Dict[str, List[str]]) -> int:
        return sum(len(p) for p in paths.values())
    # Graph Structure
    def build_graph(self, rail_map: RailMap) -> Dict[str, List[Tuple[str, float]]]:
        graph = defaultdict(list)
        for seg in rail_map.segments.values():
            s, e = seg.start_node, seg.end_node
            pts = seg.points
            cost = 0.0
            for i in range(len(pts) - 1):
                dx = pts[i+1][0] - pts[i][0]
                dy = pts[i+1][1] - pts[i][1]
                cost += (dx**2 + dy**2) ** 0.5
            graph[s].append((e, cost))
            graph[e].append((s, cost))
        return graph

    def detect_conflict(self, paths:Dict[str, List[str]]) -> Tuple[bool, Constraint]:
        max_t = max(len(p) for p in paths.values())
        for t in range(max_t):
            node_at_time = defaultdict(list)
            for agent, path, in paths.items():
                if t < len(path):
                    node = path[t]
                    node_at_time[node].append(agent)
                for node, agents in node_at_time.items():
                    if len(agents) > 1:
                        return True, Constraint(agents[1], t, node)
        return False, None
    # Get Closed Node
    def get_closest_node(self, position: Tuple[float, float]) -> str:
        def euclidean(a, b):
            return ((a[0] - b[0])**2 + (a[1]-b[1])**2)**0.5
        closest = None
        min_dist = float('inf')
        for node in self.rail_map.nodes.values():
            dist = euclidean((node.x, node.y), position)
            if dist < min_dist:
                closest = node.id
                min_dist = dist
        return closest

    # PLANNER -> GLOBAL PATH PLANNER
    def dijkstra(self, start: str, goal: str, constraints: List[Constraint], agent: str) -> List[str]:
        queue = [(0, start, [])]
        visited = set() # Form the visited set
        constraint_lookup = {(c.time, c.node) for c in constraints if c.agent == agent}
        while queue:
            cost, current, path = heapq.heappop(queue)
            if (len(path), current) in constraint_lookup:
                continue
            if current in visited:
                continue
            visited.add(current)
            path = path + [current]
            if current == goal:
                return path
            for neighbor, edge_cost in self.graph[current]:
                if neighbor not in visited:
                    heapq.heappush(queue, (cost + edge_cost, neighbor, path))
        return []

    # Create the Basic Paths
    def plan_paths(self)-> Dict[str, List[str]]:
        root_constraints = []
        root_paths = {
            agent : self.dijkstra(start, goal, root_constraints, agent)
            for agent, (start, goal) in self.vehicle_pairs.items()
        } # Basic Path
        root_cost = self.compute_cost(root_paths)
        open_list = [CBSNode(root_paths, root_constraints, root_cost)]
        while open_list:
            node = heapq.heappop(open_list)
            conflict, constraint = self.detect_conflict(node.paths)
            if not conflict:
                return node.paths
            for agent in [constraint.agent]:
                new_constraints = node.constraints + [constraint]
                new_paths = node.paths.copy()
                start, goal = self.vehicle_pairs[agent]
                new_paths[agent] = self.dijkstra(start, goal, new_constraints, agent)
                if not new_paths[agent]:
                    continue
                new_cost = self.compute_cost(new_paths)
                new_node = CBSNode(new_paths, new_constraints, new_cost)
                heapq.heappush(open_list, new_node)
        return {}

class HLCPlanner:
    def __init__(self, rail_map:RailMap):
        self.rail_map = rail_map
        self.user_list = []
        self.charger_list = []
        self.paths = {}


    def update_users(self, new_users:List[Dict]):
        """
        Args: New Users
        return : Updated Waiting Queue of User List
        """
        self.user_list.extend(new_users)

    def update_chargers(self, charger_states:List[Dict]):
        """
        Args : Charger State Message
        Return : Charger List
        """
        self.charger_list = charger_states

    def check_and_pair(self):
        idle_chargers = [c for c in self.charger_list if c.get("status") == "idle"]
        if not idle_chargers or not self.user_list:
            return
        
        # Hungarian Pairing
        pairer = HungarianPair(self.user_list, idle_chargers)
        pairs, waiting_queue = pairer.user_ev_pair()
        if not pairs:
            return

        # Path Planning
        planner = CBSPlanner(self.rail_map, pairs, self.user_list, self.charger_list)
        self.paths = planner.plan_paths()

        # Dispatch and update
        dispatch_to_vehicles(self.paths)
        self.user_list = [u for u in self.user_list if u['user_id'] in waiting_queue]

# USER MANAGER -> Subscribing Message and Update the User list
class UserManager(Node):
    def __init__(self, hlc:HLCPlanner):
        super().__init__('user_manager')
        self.user_states = dict()
        self.create_subscription(UserRequest, '/user_states', self.state_callback, 10)
        self.hlc = hlc

    def state_callback(self, msg):
        self.user_states[msg.user_id] = {
            "user_id": msg.user_id,
            "location": msg.location,
            "request_time" : msg.request_time
        }
        self.hlc.update_users(list(self.user_states.values()))
        self.hlc.check_and_pair()

# CHARGER MANAGER -> Subscribing Message and Updat the Charger List and Status
class ChargerManager(Node):
    def __init__(self, hlc:HLCPlanner):
        super().__init__('charger_manager')
        self.charger_states = dict()
        self.create_subscription(ChargerState, '/charger_states', self.state_callback, 10)
        self.hlc = hlc
    def state_callback(self, msg):
        self.charger_states[msg.charger_id] = {
            "charger_id" : msg.charger_id,
            "location" : msg.location,
            "status" : msg.status
        }
        self.hlc.update_chargers(list(self.charger_states.values()))
        self.hlc.check_and_pair()

def dispatch_to_vehicles(paths: Dict[str, List[str]]):
    print("\n[Final Collision-Free Paths]")
    for v, p in paths.items():
        print(f"{v} -> {p}")


def main(args=None):
    rclpy.init(args=args)

    rail_map = load_rail_map_from_json()
    hlc = HLCPlanner(rail_map)

    user_manager = UserManager(hlc)
    charger_manager = ChargerManager(hlc)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(user_manager)
    executor.add_node(charger_manager)

    try:
        executor.spin()  # ✅ 계속 실행되도록 ROS2 이벤트 루프 진입
    finally:
        user_manager.destroy_node()
        charger_manager.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
    # pairer = HungarianPair(users, chargers)
    # pairs, queues = pairer.user_ev_pair()
    # planner = CBSPlanner(rail_map, pairs, users, chargers)
    # result = planner.plan_paths()
    # print("\n[Paired Users]")
    # print(pairs)
    # print("\n[Charger Queues]")
    # for c, q in queues.items():
    #     print(f"{c} queue: {q}")
    # print("\n[Final collision-Free Paths]")
    # for v, p in result.items():
    #     print(f"{v}->{p}")