import numpy as np
import matplotlib.pyplot as plt
import csv
from shapely.geometry import Polygon
from commonroad.scenario.obstacle import DynamicObstacle
from commonroad.scenario.scenario import Scenario


def euclidian_distance(ego: DynamicObstacle, obstacle: DynamicObstacle, timestep):
    ego_x = ego.state_at_time(timestep).position[0]
    ego_y = ego.state_at_time(timestep).position[1]
    obstacle_x = obstacle.state_at_time(timestep).position[0]
    obstacle_y = obstacle.state_at_time(timestep).position[1]
    return np.sqrt(np.power(obstacle_x - ego_x, 2) + np.power(obstacle_y - ego_y, 2))


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


def is_visible(car1: DynamicObstacle, car2: DynamicObstacle, timestep):
    car1_position = car1.state_at_time(timestep).position
    car2_position = car2.state_at_time(timestep).position
    car1_orientation = car1.state_at_time(timestep).orientation
    car1_to_car2_orientation = compute_orientation(car1_position, car2_position)
    # normalize orientation angle
    if car1_orientation < 0:
        car1_orientation += 2 * np.pi
    if car1_to_car2_orientation < 0:
        car1_to_car2_orientation += 2 * np.pi
    if - np.pi / 2 < car1_orientation - car1_to_car2_orientation < np.pi / 2:
        return True
    else:
        return False


def compute_relative_speed(ego: DynamicObstacle, obstacle: DynamicObstacle, timestep):
    ego_velocity = ego.state_at_time(timestep).velocity
    ego_orientation = ego.state_at_time(timestep).orientation
    obstacle_velocity = obstacle.state_at_time(timestep).velocity
    obstacle_orientation = obstacle.state_at_time(timestep).orientation
    obstacle_vector_x = obstacle_velocity * np.cos(obstacle_orientation)
    obstacle_vector_y = obstacle_velocity * np.sin(obstacle_orientation)
    # galilean velocity transformation
    ego_vector_x = - ego_velocity * np.cos(ego_orientation)
    ego_vector_y = - ego_velocity * np.sin(ego_orientation)
    # relative speed computation
    relative_speed_x = ego_vector_x + obstacle_vector_x
    relative_speed_y = ego_vector_y + obstacle_vector_y
    relative_speed = np.sqrt(np.power(relative_speed_x, 2) + np.power(relative_speed_y, 2))
    # relative speed is 0 if at both vehicles are driving 'out of reach'
    if not is_visible(ego, obstacle, timestep):
        if not is_visible(obstacle, ego, timestep):
            return 0
    return relative_speed


def compute_shape(car: DynamicObstacle, timestep):
    # get position and orientation
    x = car.state_at_time(timestep).position[0]
    y = car.state_at_time(timestep).position[1]
    # add 0.1 since CommonRoad adds 0.1 as well in mp.renderer.py at _draw_occupancy()
    w = car.obstacle_shape.width
    h = car.obstacle_shape.length
    # add pi/2 because CommonRoad 'neutral' position is horizontal
    # therefore 'width' is actually length and 'length' is actually width
    # alternative: w = length, h = width
    o = car.state_at_time(timestep).orientation + np.pi / 2
    # compute position of the four corners
    top_right_x = x - w / 2 * np.cos(o) + h / 2 * np.sin(o)
    top_right_y = y - w / 2 * np.sin(o) - h / 2 * np.cos(o)
    top_right = [top_right_x, top_right_y]
    top_left_x = x + w / 2 * np.cos(o) + h / 2 * np.sin(o)
    top_left_y = y + w / 2 * np.sin(o) - h / 2 * np.cos(o)
    top_left = [top_left_x, top_left_y]
    bottom_left_x = x + w / 2 * np.cos(o) - h / 2 * np.sin(o)
    bottom_left_y = y + w / 2 * np.sin(o) + h / 2 * np.cos(o)
    bottom_left = [bottom_left_x, bottom_left_y]
    bottom_right_x = x - w / 2 * np.cos(o) - h / 2 * np.sin(o)
    bottom_right_y = y - w / 2 * np.sin(o) + h / 2 * np.cos(o)
    bottom_right = [bottom_right_x, bottom_right_y]
    # create and return shape
    return Polygon([top_right, top_left, bottom_left, bottom_right])


def is_collision(ego: DynamicObstacle, obstacle: DynamicObstacle, timestep, plot=False):
    ego_shape = compute_shape(ego, timestep)
    obstacle_shape = compute_shape(obstacle, timestep)
    if plot is True:
        fig, axs = plt.subplots()
        axs.set_aspect('equal', 'datalim')
        xs, ys = ego_shape.exterior.xy
        axs.fill(xs, ys, alpha=0.5, fc='r', ec='none')
        xs, ys = obstacle_shape.exterior.xy
        axs.fill(xs, ys, alpha=0.5, fc='b', ec='none')
        plt.show()
    return ego_shape.exterior.intersects(obstacle_shape.exterior)


def danger_metric(ego: DynamicObstacle, obstacle: DynamicObstacle, timestep):
    distance = euclidian_distance(ego, obstacle, timestep)
    relative_speed = compute_relative_speed(ego, obstacle, timestep)
    if is_collision(ego, obstacle, timestep):
        k = 100
        return True, relative_speed + k
    else:
        return False, relative_speed / distance


def safe_danger_values(scenario: Scenario, ego: DynamicObstacle, txt_path, timestep):
    for i in range(len(scenario.obstacles)):
        if scenario.obstacles[i].state_at_time(timestep) is not None:
            danger = danger_metric(ego, scenario.obstacles[i], timestep)
        else:
            danger = (False, 0.0)
        # timestep, obstacle, danger, crash
        tmp_txt = [str(timestep).zfill(2),
                   str(scenario.obstacles[i].obstacle_id),
                   str(danger[1]),
                   str(danger[0])]
        with open(txt_path, 'a') as danger_txt:
            csv.writer(danger_txt).writerow(tmp_txt)


def return_danger_values(scenario: Scenario, ego: DynamicObstacle, timestep):
    result = []
    for i in range(len(scenario.obstacles)):
        if scenario.obstacles[i].state_at_time(timestep) is not None:
            danger = danger_metric(ego, scenario.obstacles[i], timestep)
        else:
            danger = (False, 0.0)
        # timestep, obstacle, danger, crash
        tmp_result = [str(timestep).zfill(2),
                      str(scenario.obstacles[i].obstacle_id),
                      str(danger[1]),
                      str(danger[0])]
        result.append(tmp_result)
    return result
