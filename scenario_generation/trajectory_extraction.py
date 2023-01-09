import math
import numpy as np
import json

from commonroad.geometry.shape import Rectangle
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.scenario.trajectory import Trajectory, State
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.common.file_writer import CommonRoadFileWriter
from commonroad.common.file_writer import OverwriteExistingFile


def compute_orientation(initial_point: [], terminal_point: []):
    # orientation from initial_point to terminal_point
    x = (terminal_point[0] - initial_point[0])
    y = (terminal_point[1] - initial_point[1])
    # special cases
    if x < 0 and y == 0:
        return np.pi
    if x > 0 and y == 0:
        return 0
    if x == 0 and y < 0:
        return np.pi * 3 / 2
    if x == 0 and y > 0:
        return np.pi / 2
    if x == 0 and y == 0:
        raise ValueError('invalid orientation computation')
    # compute orientation
    orientation = abs(np.arctan(y / x))
    # adjust orientation according to quadrant
    if x > 0 and y > 0:
        pass
    elif x < 0 and y > 0:
        orientation = np.pi - orientation
    elif x < 0 and y < 0:
        orientation = np.pi + orientation
    elif x > 0 and y < 0:
        orientation = 2 * np.pi - orientation
    return orientation


def euclidianDistance(point1: [], point2: []):
    return math.sqrt(math.pow(point1[0] - point2[0], 2) + math.pow(point1[1] - point2[1], 2))


def computeDistance(x_coordinates: [], y_coordinates: []):
    assert len(x_coordinates) == len(y_coordinates)
    distance = 0
    for i in range(len(x_coordinates) - 1):
        distance += euclidianDistance([x_coordinates[i], y_coordinates[i]], [x_coordinates[i+1], y_coordinates[i+1]])
    return distance


def extractTrajectories(trajectory_file_path):
    vehicle_trajectories = []
    with open(trajectory_file_path) as file:
        data = json.load(file)
        try:
            for v in data:
                coordinates = []
                for c in data[v]:
                    coordinates.append([c['x'], c['y']])
                vehicle_trajectories.append(coordinates)
        except KeyError:
            print('there are no trajectories in the file')
    return vehicle_trajectories


def interpolateTrajectories(trajectories: [], velocity):
    interpolated_trajectories = []
    for trajectory in trajectories:
        interpolated_trajectory = []
        # extract 'x' and 'y'
        x_coordinates = []
        y_coordinates = []
        for c in trajectory:
            x_coordinates.append(c[0])
            y_coordinates.append(c[1])
        # fit a polynomial to x and y coordinates
        coefficients = np.polyfit(np.array(x_coordinates), np.array(y_coordinates), 3)
        # start coordinates
        start_x = x_coordinates[0]
        start_y = np.polyval(coefficients, start_x)
        # extend x coordinates assuming linear movement at the end
        extension_steps = 10
        for i in range(extension_steps):
            tmp_diff = (x_coordinates[-1] - x_coordinates[-2])
            x_coordinates.append(x_coordinates[-1] + tmp_diff)
        # compute coordinates inbetween
        extended_x_coordinates = []
        precision = 100
        for i in range(len(x_coordinates) - 1):
            extended_x_coordinates.extend(np.linspace(x_coordinates[i], x_coordinates[i + 1], precision, endpoint=False))
        extended_x_coordinates.append(x_coordinates[-1])
        extended_y_coordinates = [np.polyval(coefficients, x) for x in extended_x_coordinates]
        # compute x and y coordinates from start to end with respect to the velocity
        distance_per_timestep = 0.1 * velocity
        interpolated_trajectory.append([start_x, start_y])
        # setup trajectory start
        current_x = start_x
        current_y = start_y
        distance_to_end = computeDistance(extended_x_coordinates, extended_y_coordinates)
        index = 1
        while distance_per_timestep <= distance_to_end:
            current_distance_sum = 0
            while current_distance_sum < distance_per_timestep:
                tmp_distance = euclidianDistance([current_x, current_y], [extended_x_coordinates[index], extended_y_coordinates[index]])
                current_distance_sum += tmp_distance
                current_x = extended_x_coordinates[index]
                current_y = extended_y_coordinates[index]
                index += 1
            interpolated_trajectory.append([current_x, current_y])
            distance_to_end = computeDistance(extended_x_coordinates[index-1:], extended_y_coordinates[index-1:])
        interpolated_trajectories.append(interpolated_trajectory)
    return interpolated_trajectories


def addVehiclesToScenario(roads_file_path, interpolated_trajectories: [], velocity):
    scenario, planning_problem_set = CommonRoadFileReader(roads_file_path).open()
    for trajectory in interpolated_trajectories:
        initial_orientation = compute_orientation(trajectory[0], trajectory[1])
        initial_state = State(position=trajectory[0], velocity=velocity, orientation=initial_orientation, time_step=0)
        state_list = []
        for i in range(1, len(trajectory) - 1):
            new_position = trajectory[i]
            new_orientation = compute_orientation(trajectory[i], trajectory[i + 1])
            new_state = State(position=new_position, velocity=velocity, orientation=new_orientation, time_step=i)
            state_list.append(new_state)
        obstacle_trajectory = Trajectory(1, state_list)
        obstacle_shape = Rectangle(length=4.3, width=1.8)
        obstacle_prediction = TrajectoryPrediction(obstacle_trajectory, obstacle_shape)
        obstacle_id = scenario.generate_object_id()
        obstacle_type = ObstacleType.CAR
        obstacle = DynamicObstacle(obstacle_id, obstacle_type, obstacle_shape, initial_state, obstacle_prediction)
        scenario.add_objects(obstacle)
    return scenario, planning_problem_set


def writeToFile(output_file_path, scenario, planning_problem_set):
    author = 'Franz Scheuer'
    affiliation = 'University of Passau'
    fw = CommonRoadFileWriter(scenario, planning_problem_set, author=author, affiliation=affiliation)
    fw.write_to_file(output_file_path, OverwriteExistingFile.ALWAYS)


def extractVehicles(trajectory_file_path, roads_file_path, output_file_path, velocity):
    trajectories = extractTrajectories(trajectory_file_path)
    interpolated_trajectories = interpolateTrajectories(trajectories, velocity)
    new_scenario, new_planning_problem_set = addVehiclesToScenario(roads_file_path, interpolated_trajectories, velocity)
    writeToFile(output_file_path, new_scenario, new_planning_problem_set)
