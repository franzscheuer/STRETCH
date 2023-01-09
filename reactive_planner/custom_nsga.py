import copy
import random
from typing import List
import numpy as np
import json

from scenario_extension import extend_scenario, reduce_scenario
from run_combined_planner import run_planner

from jmetal.core.problem import FloatProblem
from jmetal.core.solution import FloatSolution
from jmetal.core.operator import Mutation, Crossover


class CustomFloatProblem(FloatProblem):

    def __init__(self, filename: str, parameters: [], time_interval: [float, float], parameter_interval: [], population_size: int):
        super(CustomFloatProblem, self).__init__()
        self.filename = filename
        self.file_id = filename[(filename.rfind('/')+1):-4]
        self.information_directory = f'./information_files/{self.file_id}'
        self.parameters = parameters
        # generation information
        self.population_size = population_size
        self.generation_number = 0
        self.individual_counter = 0
        # (1) minimize: scenario extension time
        # (2) minimize: max. danger new - max. danger original
        # (3) minimize: difference in cost function parameters (normalize euclidian distance in each dimension for the weights)
        self.number_of_objectives = 3
        self.directions = [self.MINIMIZE, self.MINIMIZE, self.MINIMIZE]
        # (1) extension time
        # (2)-(9) cost function parameters
        self.number_of_variables = 9
        self.number_of_constraints = 0
        self.time_interval = time_interval
        self.lower_bound = [time_interval[0]]
        self.lower_bound.extend(parameter_interval[0])
        self.upper_bound = [time_interval[1] + 5.0] if time_interval[1] != -1000.0 else [time_interval[0] + 5.0]
        self.upper_bound.extend(parameter_interval[1])
        self.labels = ['extension_time', 'danger_diff', 'parameter_diff']
        self.all_solutions = []
        # setup reference scenario
        self.ref_result = self.setup_ref_result(filename)

    def setup_ref_result(self, filename: str):
        result = run_planner(filename, self.parameters, save_information=True, information_directory=self.information_directory,
                             max_sampling=False, true_vision=True, safety_buffer=True)
        attributes = {'crash': bool(result['crash']),
                      'goal_reached': bool(result['goal_reached']),
                      'max_danger': result['max_danger'],
                      'scenario_length': result['scenario_length'],
                      'lower_bound': self.time_interval[0],
                      'upper_bound': self.time_interval[1]}
        ref_file_path = f'{self.information_directory}/{self.file_id}.json'
        with open(ref_file_path, 'r') as ref_file:
            data = json.load(ref_file)
        data[f'{self.parameters}']['attributes'] = attributes
        with open(ref_file_path, 'w') as ref_file:
            json.dump(data, ref_file)
        return result

    def create_solution(self) -> FloatSolution:
        new_solution = FloatSolution(self.lower_bound, self.upper_bound, self.number_of_objectives,
                                     self.number_of_constraints)
        new_time = random.uniform(self.lower_bound[0], self.upper_bound[0])
        parameter_upper = self.upper_bound[1:]
        parameter_lower = self.lower_bound[1:]
        new_parameters = []
        for p in self.parameters:
            for _ in p:
                new_parameters.append(random.uniform(parameter_lower.pop(), parameter_upper.pop()))
        new_solution.variables = [new_time]
        new_solution.variables.extend(new_parameters)
        return new_solution

    def evaluate(self, solution: FloatSolution) -> FloatSolution:
        pla_r = obs_r = 0.0
        extension_time = round(solution.variables[0], 1)
        cost_variables = solution.variables[1:]
        cost_parameters = []
        for p in self.parameters:
            new_parameter = []
            for _ in p:
                new_parameter.append(cost_variables.pop())
            cost_parameters.append(new_parameter)
        if extension_time > 0.0:
            tmp_filename = f'./example_scenarios/{self.file_id}/extensions/t_{extension_time}_pla_r_{pla_r}_obs_r_{obs_r}.xml'
            extend_scenario(self.filename, tmp_filename, extension_time, pla_r, obs_r)
        elif extension_time < 0.0:
            tmp_filename = f'./example_scenarios/{self.file_id}/reductions/t_{abs(extension_time)}.xml'
            reduce_scenario(self.filename, tmp_filename, abs(extension_time))
        else:
            tmp_filename = f'./example_scenarios/{self.file_id}/reductions/t_0.0.xml'
        result = run_planner(tmp_filename, cost_parameters, save_information=True, information_directory=self.information_directory,
                             max_sampling=False, true_vision=True, safety_buffer=True)
        solution.objectives[0] = extension_time
        solution.objectives[1] = result['max_danger'] - self.ref_result['max_danger']
        # compute normalized euclidean distance
        assert len(self.parameters) == len(cost_parameters)
        euclidean_distance = 0
        tmp_upper = self.upper_bound[1:]
        tmp_lower = self.lower_bound[1:]
        for i in range(len(self.parameters)):
            assert len(self.parameters[i]) == len(cost_parameters[i])
            for k in range(len(self.parameters[i])):
                weight_diff = self.parameters[i][k] - cost_parameters[i][k]
                search_diff = tmp_upper.pop() - tmp_lower.pop()
                euclidean_distance += np.square(weight_diff / search_diff)
        solution.objectives[2] = np.sqrt(euclidean_distance)
        # add crash information
        solution.attributes['crash'] = True if result['max_danger'] == 200 else result['crash']
        solution.attributes['goal_reached'] = bool(result['goal_reached'])
        solution.attributes['scenario_length'] = result['scenario_length']
        # add attributes to information file
        attributes = {'generation_number': self.generation_number,
                      'individual_number': self.individual_counter,
                      'crash': solution.attributes['crash'],
                      'goal_reached': solution.attributes['goal_reached'],
                      'scenario_length': solution.attributes['scenario_length']}
        self.individual_counter += 1
        if self.individual_counter % self.population_size == 0:
            self.generation_number += 1
        # add objectives to information file
        objectives = {'extension_time': solution.objectives[0],
                      'danger_difference': solution.objectives[1],
                      'parameter_difference': solution.objectives[2]}
        curr_file_id = tmp_filename[(tmp_filename.rfind('/')+1):-4]
        information_file_path = f'{self.information_directory}/{curr_file_id}.json'
        with open(information_file_path, 'r') as information_file:
            data = json.load(information_file)
        data[f'{cost_parameters}']['objectives'] = objectives
        data[f'{cost_parameters}']['attributes'] = attributes
        with open(information_file_path, 'w') as information_file:
            json.dump(data, information_file)
        self.all_solutions.append(solution)
        return solution

    def get_all_solutions(self) -> []:
        return self.all_solutions

    def get_ref_result(self):
        return self.ref_result

    def get_name(self) -> str:
        return 'Danger Minimization'


class CustomMutation(Mutation[FloatSolution]):
    def __init__(self, probability: float, distribution_index: float):
        super(CustomMutation, self).__init__(probability=probability)
        self.distribution_index = distribution_index

    def execute(self, solution: FloatSolution) -> FloatSolution:
        for i in range(solution.number_of_variables):
            rand = random.random()
            if rand <= self.probability:
                y = solution.variables[i]
                yl, yu = solution.lower_bound[i], solution.upper_bound[i]
                if yl == yu:
                    y = yl
                else:
                    delta1 = (y - yl) / (yu - yl)
                    delta2 = (yu - y) / (yu - yl)
                    rnd = random.random()
                    mut_pow = 1.0 / (self.distribution_index + 1.0)
                    if rnd <= 0.5:
                        xy = 1.0 - delta1
                        val = 2.0 * rnd + (1.0 - 2.0 * rnd) * (pow(xy, self.distribution_index + 1.0))
                        delta_q = pow(abs(val), mut_pow) - 1.0
                    else:
                        xy = 1.0 - delta2
                        val = 2.0 * (1.0 - rnd) + 2.0 * (rnd - 0.5) * (pow(xy, self.distribution_index + 1.0))
                        delta_q = 1.0 - pow(abs(val), mut_pow)
                    y += delta_q * (yu - yl)
                    if y < solution.lower_bound[i]:
                        y = solution.lower_bound[i]
                    if y > solution.upper_bound[i]:
                        y = solution.upper_bound[i]
                solution.variables[i] = y
        return solution

    def get_name(self):
        return 'Custom Mutation'


class CustomCrossover(Crossover[FloatSolution, FloatSolution]):
    __EPS = 1.0e-14

    def __init__(self, probability: float, distribution_index: float):
        super(CustomCrossover, self).__init__(probability=probability)
        self.distribution_index = distribution_index

    def execute(self, parents: List[FloatSolution]) -> List[FloatSolution]:
        offspring = [copy.deepcopy(parents[0]), copy.deepcopy(parents[1])]
        rand = random.random()
        if rand <= self.probability:
            for i in range(parents[0].number_of_variables):
                value_x1, value_x2 = parents[0].variables[i], parents[1].variables[i]
                if random.random() <= 0.5:
                    if abs(value_x1 - value_x2) > self.__EPS:
                        if value_x1 < value_x2:
                            y1, y2 = value_x1, value_x2
                        else:
                            y1, y2 = value_x2, value_x1
                        lower_bound, upper_bound = parents[0].lower_bound[i], parents[1].upper_bound[i]
                        beta = 1.0 + (2.0 * (y1 - lower_bound) / (y2 - y1))
                        alpha = 2.0 - pow(beta, -(self.distribution_index + 1.0))
                        rand = random.random()
                        if rand <= (1.0 / alpha):
                            beta_q = pow(abs(rand * alpha), 1.0 / (self.distribution_index + 1.0))
                        else:
                            beta_q = pow(abs(1.0 / (2.0 - rand * alpha)), 1.0 / (self.distribution_index + 1.0))
                        c1 = 0.5 * (y1 + y2 - beta_q * (y2 - y1))
                        beta = 1.0 + (2.0 * (upper_bound - y2) / (y2 - y1))
                        alpha = 2.0 - pow(beta, -(self.distribution_index + 1.0))
                        if rand <= (1.0 / alpha):
                            beta_q = pow(abs((rand * alpha)), 1.0 / (self.distribution_index + 1.0))
                        else:
                            beta_q = pow(abs(1.0 / (2.0 - rand * alpha)), 1.0 / (self.distribution_index + 1.0))
                        c2 = 0.5 * (y1 + y2 + beta_q * (y2 - y1))
                        if c1 < lower_bound:
                            c1 = lower_bound
                        if c2 < lower_bound:
                            c2 = lower_bound
                        if c1 > upper_bound:
                            c1 = upper_bound
                        if c2 > upper_bound:
                            c2 = upper_bound
                        if random.random() <= 0.5:
                            offspring[0].variables[i] = c2
                            offspring[1].variables[i] = c1
                        else:
                            offspring[0].variables[i] = c1
                            offspring[1].variables[i] = c2
                    else:
                        offspring[0].variables[i] = value_x1
                        offspring[1].variables[i] = value_x2
                else:
                    offspring[0].variables[i] = value_x1
                    offspring[1].variables[i] = value_x2
        return offspring

    def get_number_of_parents(self) -> int:
        return 2

    def get_number_of_children(self) -> int:
        return 2

    def get_name(self) -> str:
        return 'Custom Crossover'
