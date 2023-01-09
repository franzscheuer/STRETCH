import csv
import os
import numpy as np

from commonroad.common.file_reader import CommonRoadFileReader

from scenario_extension import extend_scenario, reduce_scenario, reduction_is_possible
from search import binary_crash_search, fast_search, nsga_II


# standard cost parameters
acceleration_costs = [5]
velocity_costs = [5, 50, 100]
distance_costs = [0.25, 20]
orientation_costs = [0.25, 5]
cost_parameters = [acceleration_costs, velocity_costs, distance_costs, orientation_costs]

# fault based planners
only_acc = [[5], [0, 0, 0], [0, 0], [0, 0]]
only_vel = [[0], [5, 50, 100], [0, 0], [0, 0]]
only_dis = [[0], [0, 0, 0], [0.25, 20], [0, 0]]
only_ori = [[0], [0, 0, 0], [0, 0], [0.25, 5]]
random_choice = [[0], [0, 0, 0], [0, 0], [0, 0]]


def upper_bound(scenario, lower_boundary, quick_search=False):
    scenario_id = scenario[(scenario.rfind('/')+1):-4]
    reduction_time = lower_boundary
    # search limit in negative reduction (extension) direction
    search_limit = -5.0
    continue_search = True
    while continue_search is True:
        if reduction_time >= 0.0:
            # setup output filename
            reduction_filename = f'./example_scenarios/{scenario_id}/reductions/t_{round(reduction_time, 1)}.xml'
            reduce_scenario(scenario, reduction_filename, round(reduction_time, 1))
        else:
            pla_r = obs_r = 0.0
            reduction_filename = f'./example_scenarios/{scenario_id}/extensions/t_{abs(round(reduction_time, 1))}_pla_r_{pla_r}_obs_r_{obs_r}.xml'
            extend_scenario(scenario, reduction_filename, abs(round(reduction_time, 1)), pla_r, obs_r)
        # setup search file
        search_file = f'./example_scenarios/{scenario_id}/search_results/t_{round(reduction_time, 1)}.csv'
        os.makedirs(os.path.dirname(search_file), exist_ok=True)
        # clear search file
        f = open(search_file, 'w')
        f.truncate()
        f.close()
        # run planner with standard sampling space, prediction knowledge and safety buffer
        if quick_search is False:
            binary_crash_search(reduction_filename, search_file, cost_parameters,
                                max_sampling=False, true_vision=True, safety_buffer=True)
        else:
            fast_search(reduction_filename, search_file, cost_parameters,
                        max_sampling=False, true_vision=True, safety_buffer=True)
        # check for 'continue_search' conditions
        with open(search_file, 'r') as file:
            search_results = csv.reader(file)
            # skip header row
            next(search_results)
            for row in search_results:
                # label = str(row[0])
                max_danger = str(row[2])
                crash = str(row[4])
                # stop search if there is a feasible solution
                if max_danger != '200' and max_danger != '-' and crash != 'True':
                    continue_search = False
        if continue_search is False:
            return round(reduction_time, 1)
        else:
            if round(reduction_time, 1) == search_limit:
                return None
            else:
                reduction_time -= 0.1


def lower_bound(scenario, max_reduction, quick_search=False):
    scenario_id = scenario[(scenario.rfind('/')+1):-4]
    reduction_time = 0.0
    continue_search = True
    while continue_search is True:
        # before reduction check for enough distance to end of lanelet
        if reduction_is_possible(scenario, reduction_time) is False:
            return round(reduction_time - 0.1, 1)
        # setup output filename
        reduction_filename = f'./example_scenarios/{scenario_id}/reductions/t_{round(reduction_time, 1)}.xml'
        reduce_scenario(scenario, reduction_filename, round(reduction_time, 1))
        # setup search file
        search_file = f'./example_scenarios/{scenario_id}/search_results/t_{round(reduction_time, 1)}.csv'
        os.makedirs(os.path.dirname(search_file), exist_ok=True)
        # clear search file
        f = open(search_file, 'w')
        f.truncate()
        f.close()
        # run planner with maximum sampling space, perfect knowledge and no safety buffer
        if quick_search is False:
            binary_crash_search(reduction_filename, search_file, cost_parameters,
                                max_sampling=True, true_vision=False, safety_buffer=False)
        else:
            fast_search(reduction_filename, search_file, cost_parameters,
                        max_sampling=True, true_vision=False, safety_buffer=False)
        # check for 'continue_search' conditions
        with open(search_file, 'r') as file:
            search_results = csv.reader(file)
            # skip header row
            next(search_results)
            for row in search_results:
                # label = str(row[0])
                max_danger = str(row[2])
                crash = str(row[4])
                # stop search if there is no feasible solution
                if max_danger == '200' or max_danger == '-' or crash == 'True':
                    continue_search = False
        if continue_search is False:
            if reduction_time == 0.0:
                return round(reduction_time, 1)
            else:
                return round(reduction_time - 0.1, 1)
        else:
            if round(reduction_time * 10) > max_reduction:
                return round(reduction_time, 1)
            else:
                reduction_time += 0.1


def extension_search(scenario_filename):
    scenario_id = scenario_filename[(scenario_filename.rfind('/')+1):-4]
    time = np.arange(0.0, 8.0, 2.0)
    # radius = [-100.0, 0.0, 100.0]
    radius = [0.0]
    for t in time:
        for pla_r in radius:
            for obs_r in radius:
                # setup output filename
                extension_filename = f'./example_scenarios/{scenario_id}/extensions/t_{t}_pla_r_{pla_r}_obs_r_{obs_r}.xml'
                extend_scenario(scenario_filename, extension_filename, t, pla_r, obs_r)
                # setup search file
                search_file = f'./example_scenarios/{scenario_id}/search_results/t_{t}_pla_r_{pla_r}_obs_r_{obs_r}.csv'
                # clear search file
                f = open(search_file, 'w')
                f.truncate()
                f.close()
                # run planner with standard sampling space, prediction knowledge and safety buffer
                binary_crash_search(extension_filename, search_file, cost_parameters,
                                    max_sampling=False, true_vision=True, safety_buffer=True)


def setup_boundaries(search_space: float):
    lower_boundary = []
    upper_boundary = []
    for p in cost_parameters:
        for v in p:
            lower_boundary.append(0.0)
            upper_boundary.append(v * search_space)
    return [lower_boundary, upper_boundary]


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

    for sf in scenario_files:
        crfr = CommonRoadFileReader(sf)
        cr_scenario, _ = crfr.open()
        if not cr_scenario.dynamic_obstacles:
            max_bound = 50
        else:
            max_bound = len(cr_scenario.dynamic_obstacles[0].prediction.trajectory.state_list)
        lb = lower_bound(sf, max_bound, quick_search=True)
        ub = upper_bound(sf, lb, quick_search=True)
        if ub is None:
            ub = 1000.0
        print('lower:', lb, 'upper:', ub)
        nsga_II(sf, cost_parameters, lower_bound=-lb, upper_bound=-ub, parameter_interval=setup_boundaries(10.0))
        # nsga_II(sf, random_choice, lower_bound=-lb, upper_bound=-ub, parameter_interval=setup_boundaries(10.0))
