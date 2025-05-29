#!/usr/bin/env python3
import numpy as np
import cvxpy as cp
from typing import List, Tuple, Dict
import networkx as nx


"""
    Message Formation
     User Message : {user_id, ev_id, location, request_time}
     Charger_lsit : {charger_id, location, queue: [user_id, ,,,]}
     Graph Info : Node, Edge
     Rail Node

"""


class GlobalPathPlanner:
    def __init__(self, graph: nx.DiGraph, vehicle_pairs: List[Tuple[str,str]], request_times:Dict[str,float]):
        self.graph = graph # node-edge graph
        self.vehicle_pairs = vehicle_pairs # List of (start_node, goal node)
        self.request_times = request_times # Dict of vehicle_id -> request_time
        self.segment_time = 1 # Assume unit time to pass a segment
        self.planning_horizon = 20 # Max Steps

    def initial_paths(self) -> Dict[str, List[str]]:
        """Use Dijkstra to compute shortest paths."""
        paths = {}
        for i, (start, goal) in enumerate(self.vehicle_pairs):
            vehicle_id = f"veh{i}" # vehicle_id -> Vehicle ID will given by Message
            path = nx.shortest_path(self.graph, start, goal, weight = 'weight')
            paths[vehicle_id] = path # Store each paths for vehicles
        return paths

    def detect_conflicts(self, paths: Dict[str, List[str]]) -> Dict[int, dict[str, str]]:
        "Detect which vehicles share the same segment at each time step"
        timeline = {}
        for v_id, path in paths.items():
            for t, seg in enumerate(path):
                if t not in timeline:
                    timeline[t] = {}
                if seg in timeline[t].values():
                    for other_id, other_seg in timeline[t].items():
                        if other_seg == seg:
                            print(f"[Conflict] {v_id} and {other_id} on segment {seg} at time {t}")
                timeline[t][v_id] = seg
        return timeline # Return Conflict Time Line

    def optimize_paths(self, paths: Dict[str, List[str]]) -> Dict[str, List[str]]:
        vehicles = list(paths.keys())
        T = self.planning_horizon
        V = len(vehicles)
        nodes = list(self.graph.nodes)
        x={
            (v, t, n): cp.Variable(boolean=True) # Vehicle, Time, Nodes
            for v in vehicles for t in range(T) for n in nodes
        }
        constraints = []

        for v in vehicles:
            start_node = paths[v][0]
            constraints.append(x[v,0,start_node] == 1)
        for v in vehicles:
            for t in range(T):
                constraints.append(cp.sum([x[v,t,n] for n in nodes]) == 1)
        for v in vehicles:
            for t in range(T-1):
                for n1 in nodes:
                    neighbors = list(self.graph.successors(n1)) + [n1]  # allow waiting
                    lhs = x[v, t, n1]
                    rhs = cp.sum([x[v, t + 1, n2] for n2 in neighbors])
                    constraints.append(lhs <= rhs)
        # Step 5: Conflict avoidance
        for t in range(T):
            for n in nodes:
                constraints.append(cp.sum([x[v, t, n] for v in vehicles]) <= 1)
        # Step 6: Objective: minimize total cost + slack if needed
        cost = 0
        for v in vehicles:
            goal_node = paths[v][-1]
            for t in range(T):
                cost += (T - t) * x[v, t, goal_node]  # Encourage early arrival

        prob = cp.Problem(cp.Minimize(-cost), constraints)  # Maximize arrival score

        print("Solving QCQP... (this may take time)")
        prob.solve(solver=cp.ECOS_BB)
       # Step 7: Extract new paths
        new_paths = {v: [] for v in vehicles}
        for v in vehicles:
            for t in range(T):
                for n in nodes:
                    if x[v, t, n].value and x[v, t, n].value > 0.5:
                        new_paths[v].append(n)
                        break
        return new_paths

# === Example ===
if __name__ == "__main__":
    import matplotlib.pyplot as plt

    G = nx.DiGraph()
    G.add_weighted_edges_from([
        ("A", "B", 1), ("B", "C", 1), ("C", "D", 1),
        ("A", "E", 1), ("E", "F", 1), ("F", "D", 1)
    ])

    planner = GlobalPathPlanner(G, [("A", "D"), ("A", "D")], {"veh0": 0, "veh1": 5})
    paths = planner.initial_paths()
    planner.detect_conflicts(paths)

    optimized = planner.optimize_paths(paths)
    print("\n[Final Paths]")
    for k, v in optimized.items():
        print(k, "->", v)

    nx.draw(G, with_labels=True)
    plt.show()
