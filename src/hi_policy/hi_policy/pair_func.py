#!/usr/bin/env python3
#Cost Function:
from typing import Tuple, List
import rclpy
def estimate_travel_time( user_info:Tuple, charger_info:Tuple):
    #TODO : Calculate the distance between user and charger
    return True
def compute_cost_detailed(user_info, charger_info, weights:List):
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
    wait = rclpy.current_time - user_info["request_time"] # current time func
    travel = estimate_travel_time(user_info["pose"], charger_info["pose"]) # cal. the distance : cost parameter 1
    charge = estimate_charge_time(user_info["battery_level"], charger_info["vehicle_type"], charger_info["charge_power"])

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

def main():
    #TODO : test Case
    pass