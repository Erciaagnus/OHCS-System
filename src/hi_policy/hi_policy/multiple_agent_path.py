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
from hlc_interfaces.msg import ChargerState, UserRequest, ChargerPath
from geometry_msgs.msg import Pose
from hi_policy.charger_publisher import Charger
from hi_policy.vehicle_visualizer import GoalVisualizer
"""
    Message Formation
     User Message : {user_id, ev_id, location, request_time}
     Charger_list : {charger_id, location, queue: [user_id, ,,,]}
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
        User_list -> [{"user_id":  , "location":  , "request_time":   }, {      }]
        Charger_list -> [{"charger_id":  , "location":  , "state" : }, {      }]

    Return:
        pairs = [ ('user_id', 'charger_id' ), ( 'user_id', 'charger_id'), (  )], charge_queues = ['user_id']
    """
    def user_ev_pair(self, dist_wgt:float=1.0, time_wgt:float = 0.1) -> List[Tuple[str, str]]:
        def euclidean(a, b):
            def get_xy(p):
                if hasattr(p, "position"):  # Pose 타입
                    return p.position.x, p.position.y
                elif hasattr(p, "x") and hasattr(p, "y"):  # RailNode 타입
                    return p.x, p.y
                elif isinstance(p, (tuple, list)):  # 튜플도 처리
                    return p[0], p[1]
                else:
                    raise TypeError("Unsupported type for euclidean distance.")

            ax, ay = get_xy(a)
            bx, by = get_xy(b)
            return ((ax - bx) ** 2 + (ay - by) ** 2) ** 0.5

        num_users = len(self.user_list) # Number of Users
        num_chargers = len(self.charger_list) # Number of Chargers
        N = max(num_users, num_chargers) # Max Num of (Users, Chargers)

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
                pairs.append((self.user_list[i]['user_id'], self.charger_list[j]['charger_id'])) # pairs = [[user_id, charger_id], [user_id, charger_id]]
                unmatched_users.discard(i)
        remaining_users = [self.user_list[i] for i in unmatched_users]
        charger_goals = [pair[1] for pair in pairs] # goal_node, goal_node id
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
        self.vehicle_ids = [f'veh{i}' for i in range(len(vehicle_pairs))] # Pair IDS
        self.vehicle_pairs = dict(zip(self.vehicle_ids, vehicle_pairs))
        self.user = {u['user_id']: u for u in user_info}
        self.charger = {c['charger_id']: c for c in charger_info}
        # Position -> Node for Vehicle Pairs
        self.vid_to_cid = {}
        for vid, (uid, cid) in zip(self.vehicle_ids, vehicle_pairs):
            goal_node = self.get_closest_node(self.user[uid]['location'])
            start_node = self.get_closest_node(self.charger[cid]['location'])
            self.vehicle_pairs[vid] = (start_node, goal_node) # charger_node, user_node
            self.vid_to_cid[vid] = cid

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
        conflicts = []
        for t in range(max_t):
            node_at_time = defaultdict(list)
            for agent, path, in paths.items():
                if t < len(path):
                    node = path[t]
                    node_at_time[node].append(agent)
                for node, agents in node_at_time.items():
                    if len(agents) > 1:
                        for a in agents[1:]:
                            conflicts.append(Constraint(a, t, node))
                        return True, Constraint(agents[1], t, node)
        return False, None
    # Get Closed Node
    def get_closest_node(self, position: Tuple[float, float]) -> str:
        def euclidean(a, b):
            def get_xy(p):
                if hasattr(p, "position"):  # Pose 타입
                    return p.position.x, p.position.y
                elif hasattr(p, "x") and hasattr(p, "y"):  # RailNode 타입
                    return p.x, p.y
                elif isinstance(p, (tuple, list)):  # 튜플도 처리
                    return p[0], p[1]
                else:
                    raise TypeError("Unsupported type for euclidean distance.")

            ax, ay = get_xy(a)
            bx, by = get_xy(b)
            return ((ax - bx) ** 2 + (ay - by) ** 2) ** 0.5

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
    def _get_charger_id_from_goal_node(self, goal_node: str) -> str:
        for cid, charger in self.charger.items():
            c_node = self.get_closest_node(charger["location"])
            if c_node == goal_node:
                return cid
        raise ValueError(f"No charger matches node {goal_node}")

    # Create the Basic Paths
    def plan_paths(self)-> Dict[str, List[str]]:
        root_constraints = []
        root_paths = {
            agent : self.dijkstra(start, goal, root_constraints, agent)
            for agent, (start, goal) in self.vehicle_pairs.items() # pairs info
        } # Basic Path
        root_cost = self.compute_cost(root_paths)
        open_list = [CBSNode(root_paths, root_constraints, root_cost)]
        while open_list:
            node = heapq.heappop(open_list)
            conflict, constraint = self.detect_conflict(node.paths)
            if not conflict:
                # 01.06 correction
                return {
                    self.vid_to_cid[vid]: path
                    for vid, path in node.paths.items()
                }
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

class HLCPlanner(Node):
    def __init__(self, rail_map:RailMap):
        super().__init__('HighLevelController')
        self.rail_map = rail_map
        self.user_list = []
        self.charger_list = []
        self.paths = {}
        self.dispatcher = PathDispatcher()
        self.user_manager = UserManager(self)
        self.charger_manager = ChargerManager(self)
        self.goal_visualizer = GoalVisualizer()
        self.paired_users = set()
        self.user_sub = self.create_subscription(UserRequest, "/user_states", self.user_callback, 10)
        self.charger_sub = self.create_subscription(ChargerState, "/charger_states", self.charger_callback, 10)

    def charger_callback(self, msg: ChargerState):
        updated = False
        for c in self.charger_list:
            if c["charger_id"] == msg.charger_id:
                c["status"] = msg.status
                updated = True
                break
        if not updated:
            self.get_logger().warn(f"🚨 Charger {msg.charger_id} not found, adding again!")

            charger_info = {
                "charger_id": msg.charger_id,
                "location": msg.location,  # 최초 위치 저장
                "status": msg.status
            }
            self.charger_list.append(charger_info)

        # # 상태가 IDLE이고 사용자 대기열이 있을 때만 페어링 시도
        if msg.status == "idle" and self.user_list:
            self.check_and_pair()

    def user_callback(self, msg: UserRequest):
        if any(u['user_id'] == msg.user_id for u in self.user_list):
            self.get_logger().warn(f"[UserRequest] Duplicate request from {msg.user_id}, ignoring.")
            return
        user_info = {
            "user_id": msg.user_id,
            "location": msg.location,
            "request_time": msg.request_time
        }
        self.get_logger().info(f"[UserRequest] Received from {msg.user_id}")
        self.user_list.append(user_info)
        self.check_and_pair()

    def update_users(self, new_users:List[Dict]):
        """
        Args: New Users
        return : Updated Waiting Queue of User List from UserManager Node
        """
        existing_ids = set(u["user_id"] for u in self.user_list)
        for user in new_users:
            if user["user_id"] not in existing_ids:
                self.user_list.append(user)

    def update_chargers(self, charger_states:List[Dict]):
        """
        Args : Charger State Message
        Return : Charger List updated by ChargerManager Node
        """
        self.charger_list = charger_states

    def check_and_pair(self):
        idle_chargers = [c for c in self.charger_list if c.get("status") == "idle"]
        if not idle_chargers or not self.user_list:
            return
        # 필터링: 이미 페어된 유저 제외
        unpaired_users = [u for u in self.user_list if u['user_id'] not in self.paired_users]
        if not unpaired_users:
            return
        # Hungarian Pairing
        pairer = HungarianPair(unpaired_users, idle_chargers)
        pairs, waiting_queue = pairer.user_ev_pair()
        # 중복 제거
        unique_user_ids = set()
        filtered_pairs = []
        if not pairs:
            return

        for uid, cid in pairs:
            if uid not in unique_user_ids:
                filtered_pairs.append((uid, cid))
                unique_user_ids.add(uid)
        pairs = filtered_pairs

        pair_ids = [f'veh{i}' for i in range(len(pairs))]
        veh_pairs = dict(zip(pair_ids, pairs))
        self.user = {u['user_id']: u for u in self.user_list}


        print(f"user dict keys: {list(self.user.keys())}")

        # Path Planning
        planner = CBSPlanner(self.rail_map, pairs, self.user_list, self.charger_list)
        self.paths = planner.plan_paths()

        for vid, (uid, cid) in veh_pairs.items():
            goal_node = planner.get_closest_node(self.user[uid]['location'])
            print(f"Visualizing User Location {goal_node}")
            self.get_logger().info(f"user dict keys: {list(self.user.keys())}")
            self.get_logger().info(f"veh_pairs: {veh_pairs}")
            self.get_logger().info(f"Visualizing User Location {goal_node}")
            self.goal_visualizer.publish_goals(goal_node, self.rail_map)

        for charger_id, path_node_ids in self.paths.items():
            # Visualizing
            path_xy = self._expand_path_to_points(path_node_ids)
            for charger in self.charger_list:
                if charger['charger_id'] == charger_id:
                    charger['status'] = 'busy'
            self.dispatcher.publishing_path(charger_id, path_xy)
        for uid, _ in pairs:
            self.paired_users.add(uid)
        self.user_list = [u for u in self.user_list if u['user_id'] in waiting_queue]

    def _extract_charger_id(self, agent_id: str) -> str:
        idx = int(agent_id.replace("veh", "")) # Pair Info ID -> Index Info
        return self.charger_list[idx]['charger_id']

    def _expand_path_to_points(self, node_ids: List[str]) -> List[Tuple[float, float]]:
        points = []
        for i in range(len(node_ids) - 1):
            start_id = node_ids[i]
            end_id = node_ids[i+1]
            seg = self._find_segment_between(node_ids[i], node_ids[i+1])
            if not seg:
                self.get_logger().warn(f"No segment between {start_id} and {end_id}")
                continue

            # 방향 판단
            if seg.start_node == start_id and seg.end_node == end_id:
                ordered_pts = seg.points
            elif seg.start_node == end_id and seg.end_node == start_id:
                ordered_pts = list(reversed(seg.points))
            else:
                self.get_logger().warn(f"Segment {seg.id} direction mismatch for {start_id} → {end_id}")
                continue

            # 중복 제거
            if points and ordered_pts[0] == points[-1]:
                points.extend(ordered_pts[1:])
            else:
                points.extend(ordered_pts)
        return points
            # if seg:
            #     if points and seg.points[0] == points[-1]:
            #         points.extend(seg.points[1:]) # Avoid Duplicate
            #     else:
            #         points.extend(seg.points)
        # return points

    def _find_segment_between(self, start_id: str, end_id: str) -> RailSegment:
        for seg in self.rail_map.segments.values():
            if (seg.start_node == start_id and seg.end_node == end_id) or \
            (seg.start_node == end_id and seg.end_node == start_id):
                return seg
        return None

class PathDispatcher(Node):
    def __init__(self):
        super().__init__('Dispatcher')
        self.publisher_ = self.create_publisher(ChargerPath, '/charger_paths', 10)
    def publishing_path(self, charger_id: str, path_points:List[Tuple[float, float]]):
        msg = ChargerPath()
        msg.charger_id = charger_id
        msg.path = [self._to_pose(x, y) for x, y in path_points]
        self.publisher_.publish(msg)
        self.get_logger().info(f"[Dispatched] {charger_id} path with {len(path_points)} points")

    def _to_pose(self, x:float, y:float) -> Pose:
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = 1.5
        return pose

# USER MANAGER -> Subscribing Message and Update the User list
class UserManager(Node):
    def __init__(self, hlc:HLCPlanner):
        super().__init__('user_manager')
        self.user_states = dict()
        self.create_subscription(UserRequest, '/user_states', self.state_callback, 10)
        self.hlc = hlc

    def state_callback(self, msg):
        if msg.user_id in self.user_states:
            return
        self.user_states[msg.user_id] = {
            "user_id": msg.user_id,
            "location": msg.location,
            "request_time" : msg.request_time
        }
        self.hlc.update_users(list(self.user_states.values()))
        #self.hlc.check_and_pair()

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
        #self.hlc.check_and_pair()

def dispatch_to_vehicles(paths: Dict[str, List[str]]):
    print("\n[Final Collision-Free Paths]")
    for v, p in paths.items():
        print(f"{v} -> {p}")


def main(args=None):
    rclpy.init(args=args)

    rail_map = load_rail_map_from_json()
    hlc = HLCPlanner(rail_map)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(hlc)
    executor.add_node(hlc.dispatcher)
    executor.add_node(hlc.user_manager)
    executor.add_node(hlc.charger_manager)
    executor.add_node(hlc.goal_visualizer)

    try:
        executor.spin()
    finally:
        hlc.destroy_node()
        hlc.dispatcher.destroy_node()
        hlc.user_manager.destroy_node()
        hlc.charger_manager.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
