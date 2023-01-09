import json
import math
import os
import xml.etree.ElementTree as ElementTree

import numpy as np

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.file_writer import CommonRoadFileWriter
from commonroad.common.file_writer import OverwriteExistingFile
from commonroad.geometry.shape import Rectangle
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.obstacle import ObstacleType, DynamicObstacle
from commonroad.scenario.trajectory import State
from commonroad.scenario.trajectory import Trajectory
from commonroad.visualization.mp_renderer import MPRenderer
from trajectory_extraction import compute_orientation
from crash_extraction import extractVehicles


def plotScenario(file_path, output_path):
    # for gif
    # ffmpeg -f image2 -framerate 10 -i plot_%02d.png SCENARIO_NUMBER.gif
    scenario, planning_problem_set = CommonRoadFileReader(file_path).open()
    steps = max(len(scenario.dynamic_obstacles[0].prediction.trajectory.state_list),
                len(scenario.dynamic_obstacles[1].prediction.trajectory.state_list))
    for i in range(0, steps + 1):
        rnd = MPRenderer()
        scenario.draw(rnd, draw_params={'time_begin': i, 'dynamic_obstacle': {'draw_shape': False},
                                        'trajectory': {'draw_trajectory': False}})

        scenario.dynamic_obstacles[0].draw(rnd,
                                           draw_params={'time_begin': i, 'trajectory': {'draw_trajectory': False},
                                                        'dynamic_obstacle': {'vehicle_shape': {'occupancy': {
                                                            'shape': {'rectangle': {'facecolor': 'r'}}}}}})

        scenario.dynamic_obstacles[1].draw(rnd,
                                           draw_params={'time_begin': i, 'trajectory': {'draw_trajectory': False},
                                                        'dynamic_obstacle': {'vehicle_shape': {'occupancy': {
                                                            'shape': {'rectangle': {'facecolor': 'b'}}}}}})

        plt_path = output_path + '/plot_' + str(i).zfill(2) + '.png'
        os.makedirs(os.path.dirname(plt_path), exist_ok=True)
        rnd.render(filename=plt_path)


def extractRoadNodes(file_path):
    road_nodes = []
    with open(file_path) as file:
        data = json.load(file)
        try:
            for r in data['roads']:
                road_nodes.append(data['roads'][r])
        except KeyError:
            print('there are no road nodes in the file')
    return road_nodes


def addRoadsToScenario(road_coordinates, empty_scenario, roads_file_path):
    xml_file = ElementTree.parse(empty_scenario)
    lanelet_id = 1

    for road in road_coordinates:
        left_boundary = road['left']
        right_boundary = road['right']
        center_line = road['center']

        assert len(left_boundary['points']) == len(right_boundary['points']) == len(center_line['points'])

        lanelets = [[left_boundary, center_line], [center_line, right_boundary]]
        for lanelet in lanelets:
            new_lanelet = ElementTree.SubElement(xml_file.getroot(), 'lanelet')
            new_lanelet.attrib['id'] = str(lanelet_id)
            lanelet_id += 1
            left_bound = ElementTree.SubElement(new_lanelet, 'leftBound')
            right_bound = ElementTree.SubElement(new_lanelet, 'rightBound')

            reverse = True
            if reverse:
                lanelet_1 = reversed(lanelet[0]['points'])
                lanelet_2 = reversed(lanelet[1]['points'])
            else:
                lanelet_1 = lanelet[0]['points']
                lanelet_2 = lanelet[1]['points']

            for p in lanelet_1:
                left_point = ElementTree.SubElement(left_bound, 'point')
                x_left = ElementTree.SubElement(left_point, 'x')
                y_left = ElementTree.SubElement(left_point, 'y')
                x_left.text = str(p[0])
                y_left.text = str(p[1])

            for p in lanelet_2:
                right_point = ElementTree.SubElement(right_bound, 'point')
                x_left = ElementTree.SubElement(right_point, 'x')
                y_left = ElementTree.SubElement(right_point, 'y')
                x_left.text = str(p[0])
                y_left.text = str(p[1])

            lanelet_type = ElementTree.SubElement(new_lanelet, 'laneletType')
            lanelet_type.text = 'unknown'

    xml_file.write(roads_file_path)


def extractVehiclesNodes(file_path):
    vehicles_nodes = []
    with open(file_path) as file:
        data = json.load(file)
        try:
            for r in data['vehicles']:
                vehicles_nodes.append(r)
        except KeyError:
            print('there are no vehicle nodes in the file')
    return vehicles_nodes


def computeVelocities(x_coordinates, y_coordinates):
    # new_position = old_position + time_step_size * velocity
    # -> velocity = (new_position - old_position) / time_step_size
    time_step_size = 0.1
    velocities = []
    assert len(x_coordinates) == len(y_coordinates)
    for i in range(len(x_coordinates) - 1):
        v = (math.sqrt(math.pow(x_coordinates[i+1] - x_coordinates[i], 2) +
                       math.pow(y_coordinates[i+1] - y_coordinates[i], 2))) / time_step_size
        velocities.append(v)
    velocities.append(velocities[-1])
    return velocities


def computeOrientations(x_coordinates, y_coordinates):
    orientations = []
    assert len(x_coordinates) == len(y_coordinates)
    for i in range(len(x_coordinates) - 1):
        curr_point = [x_coordinates[i], y_coordinates[i]]
        next_point = [x_coordinates[i + 1], y_coordinates[i + 1]]
        curr_orientation = compute_orientation(curr_point, next_point)
        orientations.append(curr_orientation)
    orientations.append(orientations[-1])
    return orientations


def addVehiclesToScenario(obstacles, road_file):
    scenario, planning_problem_set = CommonRoadFileReader(road_file).open()

    for obstacle in obstacles:
        obstacle_coordinates = obstacle['script']
        x_coordinates = [i['x'] for i in obstacle_coordinates]
        y_coordinates = [i['y'] for i in obstacle_coordinates]
        velocities = computeVelocities(x_coordinates, y_coordinates)
        orientations = computeOrientations(x_coordinates, y_coordinates)

        assert len(x_coordinates) == len(y_coordinates) == len(orientations) == len(velocities)

        state_list = []
        for i in range(len(x_coordinates)):
            new_position = np.array([x_coordinates[i], y_coordinates[i]])
            new_orientation = orientations[i]
            new_velocity = velocities[i]
            new_state = State(position=new_position, orientation=new_orientation, velocity=new_velocity, time_step=i)
            state_list.append(new_state)

        dynamic_obstacle_trajectory = Trajectory(1, state_list[1:])
        dynamic_obstacle_shape = Rectangle(width=1.8, length=4.3)
        dynamic_obstacle_prediction = TrajectoryPrediction(dynamic_obstacle_trajectory, dynamic_obstacle_shape)
        dynamic_obstacle_id = scenario.generate_object_id()
        dynamic_obstacle_type = ObstacleType.CAR
        dynamic_obstacle = DynamicObstacle(dynamic_obstacle_id, dynamic_obstacle_type, dynamic_obstacle_shape,
                                           state_list[0], dynamic_obstacle_prediction)
        scenario.add_objects(dynamic_obstacle)

    return scenario, planning_problem_set


def main():
    # intersections = ['117021', '129224']
    # no_intersections = ['99817', '122168', '171831']
    scenario_number = '99817'

    extract_roads = False
    extract_vehicles = False
    extract_crash = False
    plot = True

    base_directory = './CRISCE_PLUS/' + scenario_number + '/'
    plot_directory = base_directory + 'plots'
    input_file = base_directory + scenario_number + '-data.json'
    roads_file = base_directory + scenario_number + '_roads.xml'
    output_file = base_directory + scenario_number + '.xml'
    crash_file = base_directory + scenario_number + '_crash.xml'
    empty_scenario_path = 'empty_scenario.xml'
    author = 'Franz Scheuer'
    affiliation = 'University of Passau'

    if extract_roads:
        roads = extractRoadNodes(input_file)
        addRoadsToScenario(roads, empty_scenario_path, roads_file)
        scenario, planning_problem = CommonRoadFileReader(roads_file).open()
        fw = CommonRoadFileWriter(scenario, planning_problem, author=author, affiliation=affiliation)
        fw.write_to_file(roads_file, OverwriteExistingFile.ALWAYS)

    if extract_vehicles:
        vehicles = extractVehiclesNodes(input_file)
        new_scenario, new_planning_problem_set = addVehiclesToScenario(vehicles, roads_file)
        fw = CommonRoadFileWriter(new_scenario, new_planning_problem_set, author=author, affiliation=affiliation)
        fw.write_to_file(output_file, OverwriteExistingFile.ALWAYS)

    if extract_crash:
        extractVehicles('output_' + scenario_number + '.json', roads_file, crash_file)

    if plot:
        os.makedirs(plot_directory, exist_ok=True)
        plotScenario(output_file, plot_directory)
        # plotScenario(crash_file, plot_directory)

    print("Scenario " + scenario_number + " were successfully converted to CommonRoad scenario")


if __name__ == "__main__":
    main()
