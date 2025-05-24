#!/usr/bin/env python3
#Cost Function:
from typing import Tuple, List, Dict
from math import sqrt
from scipy.optimize import linear_sum_assignment
import rclpy
from rclpy.clock import Clock

def euclidean_distance(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    """
    helper function to calculate Euclidean distance between two points
    Args:
        p1 (Tuple[float, float]): first point (x, y)
        p2 (Tuple[float, float]): second point (x, y)
    Returns:
        float: distance
    """
    return ((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2) ** 0.5

def estimate_travel_time(ev_location: Tuple[float, float], charger_location: Tuple[float, float], speed: float = 60.0) -> float:
    """
    functions to estimate travel time from EV to charger based on distance and speed
    Args:
        ev_location (Tuple[float, float]): vehicle location
        charger_location (Tuple[float, float]): charger location
        speed (float): speed in m/min (default: 60)
    Returns:
        float: travel time in minutes
    """
    distance = euclidean_distance(ev_location, charger_location)
    return distance / speed

# battery capacity reference table
battery_capacity_by_type = {
    "EV6": 77.4,
    "IONIQ5": 72.6,
    "Model3": 60.0
}

def estimate_charge_time(battery_level: float, vehicle_type: str, charger_power: float) -> float:
    """
    functions to estimate charging time
     based on battery level, vehicle type, and charger power
    Args:
        battery_level (float): current battery level in percentage
        vehicle_type (str): model name of the vehicle
        charger_power (float): charging power in kW
    Returns:
        float: estimated charging time in minutes
    """
    capacity = battery_capacity_by_type.get(vehicle_type, 70.0)
    needed_kWh = (100 - battery_level) / 100 * capacity
    return (needed_kWh / charger_power) * 60

# cost function to compare the costs
def compute_cost_detailed(user_info, charger_info, weights: Dict[str, float]):
    """
    functions to compare the costs
     criteria of cost :
     Messaging Data file
        User -> data : {id, pose, request_time}
        Charger -> : {id, pose, }
    Args:
        ev (_type_): _description_
        charger (_type_): _description_
        weights (_type_): _description_

    Returns:
        _type_: _description_
    """
    #TODO :
    # user:  request_time
    # central computer : current_time - request_time -> wait time
    clock = Clock()
    now = clock.now().nanoseconds / 1e9  # seconds
    wait = now - user_info["request_time"]
    wait /= 60.0  # convert to minutes # current time func
    travel = estimate_travel_time(user_info["pose"], charger_info["pose"]) # cal. the distance : cost parameter 1
    charge = estimate_charge_time(user_info["battery_level"], user_info["vehicle_type"], charger_info["charge_power"])

    total = (
        weights["wait_time"] * wait +
        weights["travel_time"] * travel +
        weights["charge_time"] * charge
    )

    return {
        "total_cost": total,
        "components": {
            "wait_time": wait,
            "travel_time": travel,
            "charge_time": charge
        }
    }

#Pairing Function:
from scipy.optimize import linear_sum_assignment

def run_pairing_detailed(ev_list, charger_list, weights):
    """

    Args:
        ev_list (_type_): _description_
        charger_list (_type_): _description_
        weights (_type_): _description_

    Returns:
        _type_: _description_
        eg. []
    """
    cost_matrix = []
    breakdown_matrix = []

    for ev in ev_list:
        row = []
        row_detail = [] #
        for charger in charger_list:
            cost_info = compute_cost_detailed(ev, charger, weights)
            row.append(cost_info["total_cost"])
            row_detail.append(cost_info)
        cost_matrix.append(row)
        breakdown_matrix.append(row_detail)

    row_ind, col_ind = linear_sum_assignment(cost_matrix)

    result = []
    for i, j in zip(row_ind, col_ind):
        pair = {
            "ev_id": ev_list[i]["id"],
            "charger_id": charger_list[j]["id"],
            "total_cost": breakdown_matrix[i][j]["total_cost"],
            "components": breakdown_matrix[i][j]["components"]
        }
        result.append(pair)

    return result

# main test function
def main():
    """
    functions to test pairing algorithm with sample data
     criteria of cost :
     Messaging Data file
        User -> data : {id, pose, request_time, battery_level, vehicle_type}
        Charger -> data : {id, pose, charge_power}
    Args:
        None

    Returns:
        None
    """

    EVS = [
        {
            "id": "EV_1",
            "pose": (0, 0),
            "request_time": 0.0,
            "battery_level": 30.0,
            "vehicle_type": "EV6"
        },
        {
            "id": "EV_2",
            "pose": (10, 5),
            "request_time": 0.0,
            "battery_level": 50.0,
            "vehicle_type": "Model3"
        }
    ]

    CHARGERS = [
        {
            "id": "CHG_A",
            "pose": (2, 1),
            "charge_power": 22.0
        },
        {
            "id": "CHG_B",
            "pose": (15, 6),
            "charge_power": 11.0
        }
    ]

    COST_WEIGHTS = {
        "wait_time": 0.3,
        "travel_time": 0.4,
        "charge_time": 0.3
    }

    import time
    current_time = time.time()
    for ev in EVS:
        ev["request_time"] = current_time - 60 * (EVS.index(ev) + 1)  # simulate wait time

    rclpy.init()
    result = run_pairing_detailed(EVS, CHARGERS, COST_WEIGHTS)

    from pprint import pprint
    pprint(result)
    rclpy.shutdown()