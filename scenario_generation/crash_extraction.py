import matplotlib.pyplot as plt
import numpy as np
import warnings
import json

from commonroad.geometry.shape import Rectangle
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad.scenario.trajectory import Trajectory, State
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.common.file_writer import CommonRoadFileWriter
from commonroad.common.file_writer import OverwriteExistingFile

warnings.filterwarnings('ignore')


def extractTrajectories(input_file_path):
    vehicle_trajectories = {}
    with open(input_file_path) as file:
        data = json.load(file)
        try:
            for v in data['simulation_trajectory']:
                vehicle_trajectories[v] = data['simulation_trajectory'][v]
        except KeyError:
            print('there are no simulation trajectories in the file')
    return vehicle_trajectories


def extractCoordinates(vehicle_trajectories):
    vehicle_coordinates = {}
    for v, nodes in vehicle_trajectories.items():
        x_coordinates = []
        y_coordinates = []
        for coordinate in nodes:
            x_coordinates.append(coordinate[0])
            y_coordinates.append(coordinate[1])
        vehicle_coordinates[v] = [x_coordinates, y_coordinates]
    return vehicle_coordinates


def extractVelocities(input_file_path):
    vehicle_velocities = {}
    with open(input_file_path) as file:
        data = json.load(file)
        try:
            for v in data['sim_veh_speed']:
                vehicle_velocities[v] = data['sim_veh_speed'][v]
        except KeyError:
            print('there are no simulation velocities in the file')
    return vehicle_velocities


def extractRotationVectors(input_file_path):
    vehicle_rotation_vectors = {}
    with open(input_file_path) as file:
        data = json.load(file)
        try:
            for v in data['simulation_rotations']:
                vehicle_rotation_vectors[v] = data['simulation_rotations'][v]
        except KeyError:
            print('there are no simulation rotations in the file')
    return vehicle_rotation_vectors


def extractOrientations(vehicle_rotation_vectors):
    vehicle_orientations = {}
    for v, nodes in vehicle_rotation_vectors.items():
        vehicle_orientation = []
        for rotation in nodes:
            vehicle_orientation.append((np.arctan2(rotation[1], rotation[0])))
        vehicle_orientations[v] = vehicle_orientation
    return vehicle_orientations


def compressStates(x_coordinates, y_coordinates, v_orientations, v_velocities):
    state_list = []
    # start at step 1 since step 0 is initial state
    time_step = 1
    # skip first k steps
    current_step = 50
    compression_counter = 1
    # temporary solution
    standard_velocity = 20.0

    while current_step < len(x_coordinates):
        new_position = np.array([x_coordinates[current_step], y_coordinates[current_step]])
        new_orientation = v_orientations[current_step - 1]
        # new_velocity = v_velocities[current_step - 1]
        new_state = State(position=new_position, velocity=standard_velocity, orientation=new_orientation,
                          time_step=time_step)
        state_list.append(new_state)
        time_step += 1
        # decrease step_size during the simulation
        # for int(50 / compression_counter + 20)
        # 70, 45, 36, 32, 30, 28, ... , 20
        current_step += int(50 / compression_counter + 20)
        compression_counter += 1

    return state_list


def addVehiclesToScenario(roads_file_path, vehicle_coordinates, vehicle_velocities, vehicle_orientations):
    scenario, planning_problem_set = CommonRoadFileReader(roads_file_path).open()
    for vehicle, v_coordinates in vehicle_coordinates.items():
        x_coordinates = v_coordinates[0]
        y_coordinates = v_coordinates[1]
        v_velocities = vehicle_velocities[vehicle]
        v_orientations = vehicle_orientations[vehicle]

        standard_velocity = 20.0

        dynamic_obstacle_initial_state = State(position=np.array([x_coordinates[0], y_coordinates[0]]),
                                               velocity=standard_velocity, orientation=v_orientations[0], time_step=0)
        compress_states = False

        if compress_states:
            state_list = compressStates(x_coordinates, y_coordinates, v_orientations, v_velocities)
        else:
            state_list = []
            for i in range(1, len(x_coordinates)):
                new_position = np.array([x_coordinates[i], y_coordinates[i]])
                new_orientation = v_orientations[i-1]
                new_state = State(position=new_position, orientation=new_orientation, time_step=i)
                state_list.append(new_state)

        dynamic_obstacle_trajectory = Trajectory(1, state_list)
        dynamic_obstacle_shape = Rectangle(width=1.8, length=4.3)
        dynamic_obstacle_prediction = TrajectoryPrediction(dynamic_obstacle_trajectory, dynamic_obstacle_shape)
        dynamic_obstacle_id = scenario.generate_object_id()
        dynamic_obstacle_type = ObstacleType.CAR
        dynamic_obstacle = DynamicObstacle(dynamic_obstacle_id, dynamic_obstacle_type, dynamic_obstacle_shape,
                                           dynamic_obstacle_initial_state, dynamic_obstacle_prediction)
        scenario.add_objects(dynamic_obstacle)

    plt.figure(figsize=(25, 10))
    rnd = MPRenderer()
    scenario.draw(rnd)
    planning_problem_set.draw(rnd)
    rnd.render()
    # plt.show()

    return scenario, planning_problem_set


def writeToFile(output_file_path, scenario, planning_problem_set):
    author = 'Franz Scheuer'
    affiliation = 'University of Passau'
    fw = CommonRoadFileWriter(scenario, planning_problem_set, author=author, affiliation=affiliation)
    fw.write_to_file(output_file_path, OverwriteExistingFile.ALWAYS)


def extractVehicles(input_file_path, roads_file_path, output_file_path):
    trajectories = extractTrajectories(input_file_path)
    coordinates = extractCoordinates(trajectories)
    velocities = extractVelocities(input_file_path)
    rotation_vectors = extractRotationVectors(input_file_path)
    orientations = extractOrientations(rotation_vectors)
    new_scenario, new_planning_problem_set = addVehiclesToScenario(roads_file_path, coordinates, velocities,
                                                                   orientations)
    writeToFile(output_file_path, new_scenario, new_planning_problem_set)
