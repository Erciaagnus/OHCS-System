#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from typing import List
from parking_world.rail_visualizer import RailMap, RailNode, RailSegment
from ament_index_python.packages import get_package_share_directory
from hi_policy.charger_publisher import Charger
import json
import os
import random



def load_rail_map_from_json() -> RailMap:
    pkg_path = get_package_share_directory('parking_world')
    json_path = os.path.join(pkg_path, 'maps', 'rail_map.json')
    with open(json_path, 'r') as f:
        data = json.load(f)

    rail_map = RailMap()
    for n in data["nodes"]:
        rail_map.nodes[n["id"]] = RailNode(n["id"], n["x"], n["y"])
    for s in data["segments"]:
        rail_map.segments[s["id"]] = RailSegment(
            s["id"], s["start_node"], s["end_node"], s["points"], s["type"]
        )
    return rail_map


def main(args=None):
    rclpy.init(args=args)
    rail_map = load_rail_map_from_json()
    num_chargers = 5  # 생성할 충전기 수

    # 모든 노드 ID 목록에서 무작위 초기 위치 선택
    all_node_ids = list(rail_map.nodes.keys())
    selected_nodes = random.sample(all_node_ids, num_chargers)

    chargers: List[Charger] = []
    for i, node_id in enumerate(selected_nodes):
        charger_id = f"C{i}" # C1, C2, C3, C4, C5
        # Add the Charger Node and Position and Publishing the ChargerState
        charger_node = Charger(charger_id, rail_map, node_id)
        chargers.append(charger_node) # Append Charger Node

    executor = MultiThreadedExecutor()
    # Charger Node Multi Threading
    for charger in chargers: # Every Chargers
        executor.add_node(charger)
        charger.publish_state()
    try:
        executor.spin()
    finally:
        for charger in chargers:
            charger.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()