import numpy as np
import matplotlib.pyplot as plt
from shapely.ops import nearest_points
from shapely.geometry import LineString, Point
from shapely.geometry import Polygon as ShapelyPolygon
from commonroad.scenario.obstacle import StaticObstacle
from commonroad.geometry.shape import Polygon as CommonRoadPolygon
from commonroad.scenario.trajectory import State
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.lanelet import Lanelet
from commonroad.common.util import Interval
from commonroad.planning.planning_problem import PlanningProblem, GoalRegion


def create_planning_problem(planning_problem: PlanningProblem, target_area):
    target_state = State(position=target_area, time_step=Interval(0, 100))
    new_goal_region = GoalRegion([target_state])
    new_planning_problem = PlanningProblem(planning_problem.planning_problem_id,
                                           planning_problem.initial_state,
                                           new_goal_region)
    return new_planning_problem


def create_target_area(scenario: Scenario, ref_path: [], initial_lanelet: Lanelet, adjacent_left: Lanelet, adjacent_right: Lanelet, same_lanelet: bool):
    obstacle = scenario.obstacles[0]
    if not scenario.dynamic_obstacles:
        # find crash point for static obstacle
        crash_point = Point(obstacle.initial_state.position[0], obstacle.initial_state.position[1])
    else:
        # find crash point for dynamic obstacle
        if len(obstacle.prediction.trajectory.state_list) == 1:
            obs_traj = Point(obstacle.prediction.trajectory.state_list[0].position[0], obstacle.prediction.trajectory.state_list[0].position[0])
        else:
            obs_traj = LineString([s.position for s in obstacle.prediction.trajectory.state_list])
        ego_traj = LineString(ref_path)
        crash_point = obs_traj.intersection(ego_traj)
        if crash_point.is_empty:
            # if there is no intersection find the nearest point
            crash_point = nearest_points(obs_traj, ego_traj)[0]
    left_border = initial_lanelet.left_vertices
    right_border = initial_lanelet.right_vertices
    # replace left_border with right border of adjacent lanelets with same driving direction
    if adjacent_left:
        left_border = adjacent_left.left_vertices
    # replace right_border with right border of adjacent lanelets with same driving direction
    if adjacent_right:
        right_border = adjacent_right.right_vertices
    # exclude area 'before' the crash if start and goal are on the same lanelet
    if same_lanelet is True:
        # find the closest points on left and right border
        left_cutoff_index = None
        right_cutoff_index = None
        tmp_distance = 1000
        for i in range(0, len(left_border)):
            dist = np.sqrt(np.power(left_border[i][0] - crash_point.x, 2) + np.power(left_border[i][1] - crash_point.y, 2))
            if dist < tmp_distance:
                left_cutoff_index = i
                tmp_distance = dist
        tmp_distance = 1000
        for i in range(0, len(right_border)):
            dist = np.sqrt(np.power(right_border[i][0] - crash_point.x, 2) + np.power(right_border[i][1] - crash_point.y, 2))
            if dist < tmp_distance:
                right_cutoff_index = i
                tmp_distance = dist
        goal_area = ShapelyPolygon([crash_point, *left_border[left_cutoff_index:], *right_border[:right_cutoff_index-1:-1]])
    else:
        goal_area = ShapelyPolygon([*left_border, *right_border[::-1]])
    # exclude crash area with a small safety zone
    crash_radius = 2 * max(obstacle.obstacle_shape.width, obstacle.obstacle_shape.length) * 1.5
    crash_area = crash_point.buffer(crash_radius)
    if not scenario.static_obstacles:
        # add 'trajectory buffer zone' to crash area
        car_width = obstacle.obstacle_shape.width
        obs_traj_zone = obs_traj.buffer(car_width)
        ego_traj_zone = ego_traj.buffer(car_width)
        zone_intersection = ego_traj_zone.intersection(obs_traj_zone)
    else:
        zone_intersection = ShapelyPolygon()
    # show different areas
    show = False
    if show is True:
        fig, axs = plt.subplots()
        axs.set_aspect('equal', 'datalim')
        xs, ys = crash_area.exterior.xy
        axs.fill(xs, ys, alpha=0.5, fc='r', ec='none')
        xs, ys = goal_area.exterior.xy
        axs.fill(xs, ys, alpha=0.5, fc='b', ec='none')
        if not scenario.static_obstacles:
            xs, ys = zone_intersection.exterior.xy
            axs.fill(xs, ys, alpha=0.5, fc='y', ec='none')
        plt.show()
    # create target polygon
    coordinates = []
    target_area = (goal_area.difference(crash_area)).difference(zone_intersection)
    x_coordinates, y_coordinates = target_area.exterior.coords.xy
    assert len(x_coordinates) == len(y_coordinates)
    for i in range(len(x_coordinates)):
        coordinates.append([x_coordinates[i], y_coordinates[i]])
    target_polygon = CommonRoadPolygon(np.array(coordinates))
    return target_polygon
