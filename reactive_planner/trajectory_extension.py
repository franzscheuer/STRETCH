import numpy as np
import shapely.geometry as shp
from commonroad.scenario.trajectory import Trajectory, State
from commonroad.scenario.obstacle import DynamicObstacle
from commonroad.scenario.scenario import Scenario
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.prediction.prediction import TrajectoryPrediction


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


def euclidian_distance(point1: [], point2: []):
    return np.sqrt(np.power(point1[0] - point2[0], 2) + np.power(point1[1] - point2[1], 2))


def compute_path_length(x_coordinates: [], y_coordinates: []):
    assert len(x_coordinates) == len(y_coordinates)
    distance = 0
    for i in range(len(x_coordinates) - 1):
        distance += euclidian_distance([x_coordinates[i], y_coordinates[i]], [x_coordinates[i+1], y_coordinates[i+1]])
    return distance


def convert_multi_to_linestring(geometry: shp.base.BaseGeometry):
    if isinstance(geometry, shp.MultiLineString):
        multi_coordinates = []
        for line in geometry:
            for c in line.coords:
                multi_coordinates.append(shp.Point(c))
        return shp.LineString(multi_coordinates)
    else:
        return geometry


def compute_distance_to_end(scenario: Scenario, planning_problem: PlanningProblem):
    curr_point, parallel_ref_path = find_ref_path(scenario, planning_problem.initial_state)
    # reverse parallel_ref_path to measure distance from curr_point to end
    parallel_ref_path = shp.LineString([parallel_ref_path.coords[-i] for i in range(1, len(parallel_ref_path.coords) + 1)])
    # available reduction distance
    available_reduction = parallel_ref_path.project(curr_point)
    return available_reduction


def rotate_state(curr_state: State, scenario_rotation: float, cx: float, cy: float):
    if scenario_rotation == 0.0:
        return curr_state
    else:
        x = curr_state.position[0] - cx
        y = curr_state.position[1] - cy
        curr_state.position[0] = x * np.cos(scenario_rotation) - y * np.sin(scenario_rotation)
        curr_state.position[1] = x * np.sin(scenario_rotation) + y * np.cos(scenario_rotation)
        curr_state.orientation += scenario_rotation
        return curr_state


def rotate_goal_state(goal_state: State, scenario_rotation: float, cx: float, cy: float):
    if scenario_rotation == 0.0:
        return goal_state
    else:
        x = goal_state.position.center[0] - cx
        y = goal_state.position.center[1] - cy
        goal_state.position.center[0] = x * np.cos(scenario_rotation) - y * np.sin(scenario_rotation)
        goal_state.position.center[1] = x * np.sin(scenario_rotation) + y * np.cos(scenario_rotation)
        return goal_state


def find_ref_path(scenario: Scenario, curr_state: State):
    # find lanelet and center_vertices
    lanelet_id = scenario.lanelet_network.find_lanelet_by_position([curr_state.position])[0][0]
    center_vertices = scenario.lanelet_network.find_lanelet_by_id(lanelet_id).center_vertices
    # round coordinates because shapley can't handle floating point numbers perfectly
    for c in center_vertices:
        c[0] = round(c[0], 4)
        c[1] = round(c[1], 4)
    curr_point = shp.Point(curr_state.position[0], curr_state.position[1])
    # create reference_path from center_vertices
    reference_path = shp.LineString(center_vertices)
    # find min_distance to curr_point
    min_distance = reference_path.distance(curr_point)
    # compute parallel_offset of reference path by min_distance
    left_offset = convert_multi_to_linestring(reference_path.parallel_offset(min_distance, 'left'))
    right_offset = convert_multi_to_linestring(reference_path.parallel_offset(min_distance, 'right'))
    # choose offset closer to curr_point
    if left_offset.distance(curr_point) < right_offset.distance(curr_point):
        parallel_ref_path = left_offset
    else:
        parallel_ref_path = right_offset
        # right-hand offsets are returned in reverse direction of the original linestring
        parallel_ref_path = shp.LineString([parallel_ref_path.coords[-i] for i in range(1, len(parallel_ref_path.coords) + 1)])
    return curr_point, parallel_ref_path


def compute_position(scenario: Scenario, curr_state: State, time_step_size, reduce=False):
    curr_point, parallel_ref_path = find_ref_path(scenario, curr_state)
    # move along parallel_ref_path by target_distance
    target_distance = time_step_size * curr_state.velocity
    if reduce is False:
        new_position = parallel_ref_path.interpolate(parallel_ref_path.project(curr_point) - target_distance)
    else:
        new_position = parallel_ref_path.interpolate(parallel_ref_path.project(curr_point) + target_distance)
    return np.array([new_position.x, new_position.y])


def extend_obstacle(scenario: Scenario, obstacle, extension_time, time_step_size=0.1):
    curr_state = obstacle.initial_state
    time_steps = int(extension_time * 10)
    curr_state.time_step = time_steps
    state_list = [curr_state]
    for i in range(0, time_steps):
        new_position = compute_position(scenario, curr_state, time_step_size)
        new_state = State(position=new_position,
                          velocity=curr_state.velocity,
                          orientation=compute_orientation(new_position, curr_state.position),
                          time_step=time_steps-1-i)
        state_list.insert(0, new_state)
        curr_state = state_list[0]
    # append 'old' time steps after 'new' time steps
    for s in obstacle.prediction.trajectory.state_list:
        s.time_step += time_steps
        state_list.append(s)
    # create new obstacle with new trajectory
    new_obstacle_trajectory = Trajectory(1, state_list[1:])
    new_obstacle_shape = obstacle.obstacle_shape
    new_obstacle_prediction = TrajectoryPrediction(new_obstacle_trajectory, new_obstacle_shape)
    new_obstacle_id = obstacle.obstacle_id
    new_obstacle_type = obstacle.obstacle_type
    new_obstacle = DynamicObstacle(new_obstacle_id, new_obstacle_type, new_obstacle_shape, state_list[0], new_obstacle_prediction)
    return new_obstacle


def reduce_obstacle(obstacle, reduction_time):
    if reduction_time == 0.0:
        return obstacle
    else:
        # initial state and trajectory
        state_list = obstacle.prediction.trajectory.state_list
        time_steps = int(reduction_time * 10)
        if time_steps >= len(state_list):
            return None
        else:
            # new initial state
            new_initial_state = state_list[time_steps-1]
            new_initial_state.time_step -= time_steps
            # new trajectory
            new_state_list = []
            for s in state_list[time_steps:]:
                s.time_step -= time_steps
                new_state_list.append(s)
            # create new obstacle with new trajectory
            new_obstacle_trajectory = Trajectory(1, new_state_list)
            new_obstacle_shape = obstacle.obstacle_shape
            new_obstacle_prediction = TrajectoryPrediction(new_obstacle_trajectory, new_obstacle_shape)
            new_obstacle_id = obstacle.obstacle_id
            new_obstacle_type = obstacle.obstacle_type
            new_obstacle = DynamicObstacle(new_obstacle_id, new_obstacle_type, new_obstacle_shape, new_initial_state, new_obstacle_prediction)
            return new_obstacle


def extend_planning_problem(scenario: Scenario, planning_problem: PlanningProblem, extension_time, time_step_size=0.1):
    curr_state = planning_problem.initial_state
    for _ in range(0, int(extension_time * 10)):
        tmp_position = compute_position(scenario, curr_state, time_step_size)
        tmp_state = State(position=tmp_position,
                          velocity=curr_state.velocity,
                          orientation=compute_orientation(tmp_position, curr_state.position))
        curr_state = tmp_state
    new_planning_problem = planning_problem
    new_planning_problem.initial_state.position = curr_state.position
    new_planning_problem.initial_state.orientation = curr_state.orientation
    return new_planning_problem


def reduce_planning_problem(scenario: Scenario, planning_problem: PlanningProblem, reduction_time, time_step_size=0.1):
    curr_state = planning_problem.initial_state
    for _ in range(0, int(reduction_time * 10)):
        tmp_position = compute_position(scenario, curr_state, time_step_size, reduce=True)
        tmp_state = State(position=tmp_position,
                          velocity=curr_state.velocity,
                          orientation=compute_orientation(curr_state.position, tmp_position))
        curr_state = tmp_state
    new_planning_problem = planning_problem
    new_planning_problem.initial_state.position = curr_state.position
    new_planning_problem.initial_state.orientation = curr_state.orientation
    return new_planning_problem


def compute_new_trajectory(trajectory: [], velocity):
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
    # original end coordinates
    end_x = x_coordinates[-1]
    end_y = np.polyval(coefficients, end_x)
    # extend coordinates to have enough points for higher velocities
    extension_steps = 100
    for i in range(extension_steps):
        tmp_diff = (x_coordinates[-1] - x_coordinates[-2])
        x_coordinates.append(x_coordinates[-1] + tmp_diff)
    # compute coordinates inbetween with high precision
    extended_x_coordinates = []
    precision = 100
    for i in range(len(x_coordinates) - 1):
        extended_x_coordinates.extend(np.linspace(x_coordinates[i], x_coordinates[i + 1], precision, endpoint=False))
    extended_x_coordinates.append(x_coordinates[-1])
    extended_y_coordinates = [np.polyval(coefficients, x) for x in extended_x_coordinates]
    # compute new x and y coordinates from start to end with respect to new velocity
    distance_per_timestep = 0.1 * velocity
    # setup trajectory start
    current_x = start_x
    current_y = start_y
    new_trajectory = [[current_x, current_y]]
    index = 1
    while index < extended_x_coordinates.index(end_x) or index < extended_y_coordinates.index(end_y):
        current_distance_sum = 0
        while current_distance_sum < distance_per_timestep:
            tmp_distance = euclidian_distance([current_x, current_y], [extended_x_coordinates[index], extended_y_coordinates[index]])
            current_distance_sum += tmp_distance
            current_x = extended_x_coordinates[index]
            current_y = extended_y_coordinates[index]
            index += 1
        new_trajectory.append([current_x, current_y])
    return new_trajectory


def change_velocity(obstacle: DynamicObstacle, velocity):
    if velocity == 0.0:
        return obstacle
    else:
        if velocity < 0.0:
            raise ValueError('please choose a positive velocity')
        else:
            # extract vehicle coordinates
            coordinates = [obstacle.initial_state.position]
            for s in obstacle.prediction.trajectory.state_list:
                coordinates.append(s.position)
            # compute new coordinates
            new_coordinates = compute_new_trajectory(coordinates, velocity)
            initial_orientation = compute_orientation(new_coordinates[0], new_coordinates[1])
            initial_state = State(position=new_coordinates[0], velocity=velocity, orientation=initial_orientation, time_step=0)
            state_list = []
            for i in range(1, len(new_coordinates) - 1):
                new_position = new_coordinates[i]
                new_orientation = compute_orientation(new_coordinates[i], new_coordinates[i + 1])
                new_state = State(position=new_position, velocity=velocity, orientation=new_orientation, time_step=i)
                state_list.append(new_state)
                if i == len(coordinates) - 2:
                    # append last state
                    last_state = State(position=new_coordinates[-1], velocity=velocity, orientation=new_orientation, time_step=len(new_coordinates))
                    state_list.append(last_state)
            # create new obstacle with new trajectory
            new_obstacle_trajectory = Trajectory(1, state_list)
            new_obstacle_shape = obstacle.obstacle_shape
            new_obstacle_prediction = TrajectoryPrediction(new_obstacle_trajectory, new_obstacle_shape)
            new_obstacle_id = obstacle.obstacle_id
            new_obstacle_type = obstacle.obstacle_type
            new_obstacle = DynamicObstacle(new_obstacle_id, new_obstacle_type, new_obstacle_shape, initial_state,
                                           new_obstacle_prediction)
            return new_obstacle
