#!/usr/bin/env python3
import rclpy
import rclpy.logging
from rclpy.node import Node
from hlc_interfaces.msg import ChargerPath, ChargerState
from geometry_msgs.msg import Pose
from typing import List, Dict, Tuple
from parking_world.rail_visualizer import RailNode, RailSegment, RailMap
import threading
import time
import heapq
import math

CHARGER_RADIUS = 1
class LowPolicy:
    """
    Logic for Controlling the Each Charger
    """
    def __init__(self, charger: "Charger", rail_map: RailMap, logger):
        self.logger = logger
        self.charger = charger
        self.rail_map = rail_map

    def step_charger(self):
        with self.charger.lock:
            """
            Update the charger State : charger_id, location, status
            Return: charger
            """
            # Algorihtm
            #TODO(1) : 
            charger = self.charger # charger: self
            # No Paths
            # Status Change
            if not charger.path: # No Paths : State Update
                "No Paths mean : Idle or Moving or Pre-Charging or Unplugging,,"
                #TODO State Transition -> Should add code after testing,,,
                # Paired Charger
                if self.charger.status == 'moving':
                    self.charger.status = 'pre-charging'
                elif self.charger.status == 'pre-charging':
                    self.charger.status = 'charging'
                elif self.charger.status == 'charging':
                    self.charger.status = 'unplugging'
                elif self.charger.status == 'connector_retreat':
                    self.charger.status = 'idle'
                return self.charger

            #Conflict Detected -> Rerouting or Moving Other Ways
            conflict_detected = self.detect_busy_conflict(horizon=7)

            if conflict_detected: # Conflict is detected
                for oc in self.charger.other_chargers.values(): # For Every chargers.
                    if self.charger.status == 'moving' and oc.status in ['moving']:
                        #TODO : CASE3 : Self - Moving, Other - Moving
                        """
                        The higher Priority Go Path, Lower Priorty wait a moment, Same Priority ->
                        """
                        if self.charger.priority < getattr(oc, 'priority', 1e9): # Request_time = Priority
                            continue
                        else:
                            self.charger.status = 'waiting'
                            return self.charger

                    elif self.charger.status == 'moving' and oc.status in ['pre-charging', 'charging', 'unplugging', 'connector_retreat']:
                        #TODO : CASE2 : Self - Moving, Other - Pre-Charging, Charging, Unplugging, Connector Retreat
                        """
                            Self should change the path, Others can't move anywhere
                        """
                        self.charger.status = 'Reroute_Needed'
                        return self.charger

                    elif self.charger.status == 'idle' and oc.status == 'moving':
                        #TODO : CASE1 : Self - Idle, Other - Moving
                        """
                        Same as Case2 but, the role is inveted, Idle(self) should move to free node
                        """
                        # We are Idel, let them go, should move aside
                        self.charger.status = 'moving_to_free_node'
                        free_node_id, free_node = self.find_nearby_free_nodes(charger,rail_map=self.rail_map)
                        node_path = self.find_escape_path(self.charger, free_node)
                        escape_path = self.convert_node_path_to_pose_path(node_path)
                        self.charger.path = escape_path
                        # State Update
                        x, y =self.charger.path[self.charger.path_index]
                        self.charger.pose.position.x = x
                        self.charger.pose.position.y = y
                        self.charger.path_index += 1
                        return self.charger
                return self.charger
            #TODO : Moving -> Path Update
            # No conflicts, proceed
            if self.charger.status == "moving" and charger.path_index < len(charger.path):
                x, y = charger.path[charger.path_index]
                self.charger.pose.position.x = x
                self.charger.pose.position.y = y
                self.charger.path_index += 1
                if self.charger.path_index >= len(charger.path):
                    self.charger.status = 'pre-charging'
            return self.charger # charger.id, charger.pose, charger.status
    

    def detect_busy_conflict(self, horizon: int=5, threshold : float = CHARGER_RADIUS * 2 ) ->bool:
        """
         Detect Conflict between this charger and others using both charger's future path
        """
        own_future = self.charger.path[:horizon]
        for charger_id, path in self.charger.other_chargers.items():
            if charger_id == self.charger.charger_id:
                continue
            other_future = path[:horizon]
            for i in range(min(len(own_future), len(other_future))):
                own_pose = own_future[i]
                other_pose = other_future[i]
                dist= self.distance(own_pose, other_pose)
                if dist < threshold:
                    self.logger.info(f"[Conflict] Charger {self.charger.charger_id} with {charger_id}")
                    return True
        return False


    def find_escape_path(self, free_node: RailNode) -> List[Tuple[float, float]]:
        start_node = self.charger.current_node_id
        goal_node = free_node.id

        path_node_ids = self.dijkstra(start_node, goal_node)
        path = []
        for nid in path_node_ids:
            node = self.rail_map.nodes[nid]
            path.append((node.x, node.y))
        return path
    def convert_to_pose(self, x: float, y: float) -> Pose:
        p = Pose()
        p.position.x = x
        p.position.y = y
        return p
    def dijkstra(self, start: str, goal: str) -> List[str]:
        queue = [(0, start, [])]
        visited = set()
        while queue:
            cost, current, path = heapq.heappop(queue)
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

    def convert_node_path_to_pose_path(self, node_path: List[str]) -> List[Tuple[float, float]]:
        pose_path = []
        for i in range(len(node_path)-1):
            start = node_path[i]
            end = node_path[i+1]
            matched_segments = [
                seg for seg in self.rail_map.segments.values()
                if (seg.start_node == start and seg.end_node == end) or
                (seg.start_node == end and seg.end_node == start)
            ]
            if not matched_segments:
                self.logger.warning(f"No segments found between {start} and {end}")
                continue
            segment = matched_segments[0]

            points = segment.points
            if segment.start_node == end:
                points = list(reversed(points))
            pose_path.extend(points)
        return pose_path

    def find_nearby_free_nodes(self, charger, rail_map : RailMap, threshold: float=2*CHARGER_RADIUS) -> List[RailNode]:
        occupied = set()
        for msg in self.charger.other_chargers.values():
            occupied.update((round(p.position.x, 2), round(p.position.y)) for p in msg.path)
        
        curr_pos = charger.current_pos
        min_dist = float('inf')
        best_node_id = None
        best_node = None
        free_nodes = []
        for node_id, node in rail_map.nodes.items():
            node_pos = (round(node.x, 2), round(node.y, 2))
            if node_pos in occupied:
                continue
            dist = self.distance((curr_pos.x, curr_pos.y), (node.x, node.y))
            if dist < min_dist:
                min_dist = dist
                best_node_id = node_id
                best_node = node
        return best_node_id, best_node

    def distance(self, a, b) -> float:
        if isinstance(a, Pose): a = (a.position.x, a.position.y)
        if isinstance(b, Pose): b = (b.position.x, b.position.y)
        return ((a[0] - b[0])**2 + (a[1] - b[1])**2)**0.5


def main(args=None):
    rclpy.init(args=args)
    node = LowPolicy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
