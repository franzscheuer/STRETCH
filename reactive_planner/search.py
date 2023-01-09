import csv
import os
from copy import deepcopy

from jmetal.algorithm.multiobjective.nsgaii import NSGAII
from jmetal.util.termination_criterion import StoppingByEvaluations

from run_combined_planner import run_planner
from custom_nsga import CustomFloatProblem, CustomCrossover, CustomMutation


def nsga_II(filename: str, standard_parameters: [], lower_bound: float, upper_bound: float, parameter_interval: []):
    # setup information directory
    file_id = filename[(filename.rfind('/') + 1):-4]
    information_directory = f'./information_files/{file_id}'
    if not os.path.isdir(information_directory):
        os.makedirs(information_directory)
    # clear information directory
    for file in os.scandir(information_directory):
        os.unlink(file)
    # setup problem
    population_size = 50
    problem = CustomFloatProblem(filename, standard_parameters, [lower_bound, upper_bound], parameter_interval, population_size)
    # termination criterion
    max_evaluations = 1000
    # setup algorithm
    algorithm = NSGAII(
        problem=problem,
        population_size=population_size,
        offspring_population_size=population_size,
        mutation=CustomMutation(probability=1.0 / problem.number_of_variables, distribution_index=20),
        crossover=CustomCrossover(probability=0.9, distribution_index=20),
        termination_criterion=StoppingByEvaluations(max_evaluations)
    )
    # run algorithm
    algorithm.run()


def fast_search(filename: str, output_file: str, cost_parameters: [], max_sampling: bool, true_vision: bool, safety_buffer: bool):
    # setup output_file
    header = ['-', 'parameters', 'max_danger', 'max_timestep', 'crash', 'goal_reached']
    with open(output_file, 'a') as search_csv:
        csv.writer(search_csv).writerow(header)
    # run planner
    result = run_planner(filename, cost_parameters, max_sampling=max_sampling, true_vision=true_vision, safety_buffer=safety_buffer)
    # safe results in output_file
    with open(output_file, 'a') as search_csv:
        b_values = ['base', str(result['parameters']), str(result['max_danger']),
                    str(result['max_timestep']), str(result['crash']), str(result['goal_reached'])]
        csv.writer(search_csv).writerows([b_values])


def binary_crash_search(filename: str, output_file: str, cost_parameters: [], max_sampling: bool, true_vision: bool, safety_buffer: bool):
    # setup output_file
    header = ['-', 'parameters', 'max_danger', 'max_timestep', 'crash', 'goal_reached']
    with open(output_file, 'a') as search_csv:
        csv.writer(search_csv).writerow(header)
    # setup base_values
    base_values = run_planner(filename, cost_parameters, max_sampling=max_sampling, true_vision=true_vision, safety_buffer=safety_buffer)
    # search cost_parameters: acceleration_costs, velocity_costs, distance_costs, orientation_costs
    for p in range(len(cost_parameters)):
        search_space = 100
        # setup min_values
        min_parameters = deepcopy(cost_parameters)
        min_parameters[p].clear()
        for v in cost_parameters[p]:
            min_parameters[p].append(0 * v)
        min_values = run_planner(filename, min_parameters,
                                 max_sampling=max_sampling, true_vision=true_vision, safety_buffer=safety_buffer)
        # setup max_values
        max_parameters = deepcopy(cost_parameters)
        max_parameters[p].clear()
        for v in cost_parameters[p]:
            max_parameters[p].append(search_space * v)
        max_values = run_planner(filename, max_parameters,
                                 max_sampling=max_sampling, true_vision=true_vision, safety_buffer=safety_buffer)
        # binary search on cost_parameters[p]
        progress_list = []
        if min_values['crash'] != max_values['crash']:
            progress_list.clear()
            progress_list.append(max_values)
            search_finished = False
            search_difference = search_space * 0.5
            search_space -= search_difference
            search_threshold = 0.001
            # if the threshold is directly at min or max
            emergency_threshold = 0.0001
            while not search_finished:
                modified_cost_parameters = deepcopy(cost_parameters)
                modified_cost_parameters[p].clear()
                for v in cost_parameters[p]:
                    modified_cost_parameters[p].append(search_space * v)
                current_result = run_planner(filename, modified_cost_parameters,
                                             max_sampling=max_sampling, true_vision=true_vision, safety_buffer=safety_buffer)
                progress_list.append(current_result)
                # reduce search_space
                if min_values['crash'] != current_result['crash']:
                    search_space -= (search_difference * 0.5)
                else:
                    search_space += (search_difference * 0.5)
                search_difference *= 0.5
                # if difference is too small, stop searching
                difference = abs(float(progress_list[-1]['parameters'][p][0]) - float(progress_list[-2]['parameters'][p][0])) \
                    / ((float(progress_list[-1]['parameters'][p][0]) + float(progress_list[-2]['parameters'][p][0])) / 2)
                # stop search if threshold found with precision 'search_threshold' or if 'emergency_threshold' is reached
                if (difference <= search_threshold and min_values['crash'] != current_result['crash']) or (difference <= emergency_threshold):
                    search_finished = True
        # safe search results in output_file
        with open(output_file, 'a') as search_csv:
            b_values = ['base', str(base_values['parameters']), str(base_values['max_danger']),
                        str(base_values['max_timestep']), str(base_values['crash']), str(base_values['goal_reached'])]
            l_values = ['min', str(min_values['parameters']), str(min_values['max_danger']),
                        str(min_values['max_timestep']), str(min_values['crash']), str(min_values['goal_reached'])]
            m_values = ['max', str(max_values['parameters']), str(max_values['max_danger']),
                        str(max_values['max_timestep']), str(max_values['crash']), str(max_values['goal_reached'])]
            if not progress_list:
                t_values = ['threshold', '-', '-', '-', '-', '-']
            else:
                t_values = ['threshold', str(progress_list[-1]['parameters']), str(progress_list[-1]['max_danger']),
                            str(progress_list[-1]['max_timestep']), str(progress_list[-1]['crash']),
                            str(progress_list[-1]['goal_reached'])]
            search_result = [b_values, l_values, m_values, t_values]
            csv.writer(search_csv).writerows(search_result)


def linear_search(filename: str, cost_parameters: [], max_sampling: bool, true_vision: bool, safety_buffer: bool):
    progress_list = []
    # linear search
    cost_change_factors = [0, 1, 5, 20, 100]
    # acceleration_costs, velocity_costs, distance_costs, orientation_costs
    for p in range(len(cost_parameters)):
        for f in cost_change_factors:
            modified_cost_parameters = deepcopy(cost_parameters)
            modified_cost_parameters[p].clear()
            for v in cost_parameters[p]:
                modified_cost_parameters[p].append(f * v)
            result = run_planner(filename, modified_cost_parameters,
                                 max_sampling=max_sampling, true_vision=true_vision, safety_buffer=safety_buffer)
            progress_list.append(result)
    # print results
    for k, entry in enumerate(progress_list):
        if k % len(cost_change_factors) == 0:
            print()
        print(entry)
