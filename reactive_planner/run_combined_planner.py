# standard imports
import csv
import os
import glob
import json
import time
from copy import deepcopy

# third party
import numpy as np

# commonroad-io
from commonroad.common.file_reader import CommonRoadFileReader

# commonroad_dc
from commonroad_dc.boundary.boundary import create_road_boundary_obstacle
from commonroad_dc.collision.collision_detection.pycrcc_collision_dispatch import create_collision_checker

# commonroad-route-planner
from commonroad_route_planner.route_planner import RoutePlanner

# reactive planner
from commonroad_rp.reactive_planner import ReactivePlanner
from commonroad_rp.visualization import visualize_planning_result
from vision_radius import vision_radius
from reachability import create_target_area, create_planning_problem
from avoidability import safe_danger_values, return_danger_values


def run_planner():
    
    #### MISSING CODE ####
    # for the full planner please contact the team behind CommonRoad
    # Matthias Althoff: althoff@tum.de
    # Gerald Würsching: gerald.wuersching@tum.de
        
    # setup goal region
    setup_target_area = True
    if setup_target_area is True and len(scenario.obstacles) > 0:
        try:
            goal_lanelet_id = scenario.lanelet_network.find_lanelet_by_position([planning_problem.goal.state_list[0].position.center])[0][0]
            goal_lanelet = route_planner.scenario.lanelet_network.find_lanelet_by_id(goal_lanelet_id)
            # find all adjacent lanelets with same driving direction to include them in the target area
            adjacent_left = None
            # left side
            curr_lanelet = goal_lanelet
            while curr_lanelet.adj_left_same_direction:
                adjacent_left = route_planner.scenario.lanelet_network.find_lanelet_by_id(curr_lanelet.adj_left)
                curr_lanelet = adjacent_left
            adjacent_right = None
            # right side
            curr_lanelet = goal_lanelet
            while curr_lanelet.adj_right_same_direction:
                adjacent_right = route_planner.scenario.lanelet_network.find_lanelet_by_id(curr_lanelet.adj_right)
                curr_lanelet = adjacent_right
            # create target area
            same_lanelet = True if initial_id == goal_lanelet_id else False
            target_area = create_target_area(scenario, ref_path, goal_lanelet, adjacent_left, adjacent_right, same_lanelet)
            # create new planning problem for correct visualization
            new_planning_problem = create_planning_problem(planning_problem, target_area)
            planning_problem = new_planning_problem
            # set new goal
            goal = new_planning_problem.goal
        except AttributeError:
            print('warning: not able to setup goal region')
            pass

    #### MISSING CODE ####
    # for the full planner please contact the team behind CommonRoad
    # Matthias Althoff: althoff@tum.de
    # Gerald Würsching: gerald.wuersching@tum.de

    result = {'parameters': cost_function_parameters, 'scenario_length': len(record_state_list),
              'max_danger': max_danger, 'max_timestep': max_timestep,
              'average_danger': average_danger, 'median_danger': median_danger,
              'crash': crash, 'goal_reached': goal.is_reached(x_0)}

    return result


if __name__ == "__main__":
    scenario_files = ['./example_scenarios/CD_M_Static.xml',
                      # original CRISCE scenarios
                      './example_scenarios/CRISCE_99817.xml',
                      './example_scenarios/CRISCE_105222.xml',
                      './example_scenarios/CRISCE_120305.xml',
                      './example_scenarios/CRISCE_122168.xml',
                      './example_scenarios/CRISCE_156722.xml',
                      './example_scenarios/CRISCE_171831.xml',
                      # adjusted CRISCE scenarios
                      './example_scenarios/CRISCE_M_99817.xml',
                      './example_scenarios/CRISCE_M_105222.xml',
                      './example_scenarios/CRISCE_M_105222_I1.xml',
                      './example_scenarios/CRISCE_M_105222_I2.xml',
                      './example_scenarios/CRISCE_M_105222_I3.xml',
                      './example_scenarios/CRISCE_M_105222_I4.xml',
                      './example_scenarios/CRISCE_M_122168_1.xml',
                      './example_scenarios/CRISCE_M_122168_2.xml',
                      './example_scenarios/CRISCE_M_156722.xml',
                      # adjusted TUM CommonRoad scenarios
                      './example_scenarios/CR_USA_Peach.xml',
                      './example_scenarios/CR_ZAM_Moelln.xml',
                      './example_scenarios/CR_ZAM_Tjunction_1.xml',
                      './example_scenarios/CR_ZAM_Tjunction_2.xml',
                      './example_scenarios/CR_ZAM_Zaventem.xml']

    standard = [[5], [5, 50, 100], [0.25, 20], [0.25, 5]]

    for sf in scenario_files:
        run_planner(sf)
