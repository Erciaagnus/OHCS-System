#!/usr/bin/env python3
#Cost Function: 

def compute_cost_detailed(ev, charger, weights):
    wait = ev["wait_time"]
    travel = estimate_travel_time(ev["location"], charger["location"])
    charge = estimate_charge_time(ev["battery_level"], ev["vehicle_type"], charger["charge_power"])

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
    cost_matrix = []
    breakdown_matrix = []

    for ev in ev_list:
        row = []
        row_detail = []
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