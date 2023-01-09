import numpy as np
import shapely.geometry as shp
from commonroad.scenario.trajectory import Trajectory, State
from commonroad.geometry.shape import Rectangle
from commonroad.scenario.obstacle import DynamicObstacle, StaticObstacle
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.lanelet import Lanelet
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad_dc.collision.collision_detection.pycrcc_collision_dispatch import create_collision_checker


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


def compute_shape(curr_state: State, obs_shape: Rectangle):
    # get position and orientation
    x = curr_state.position[0]
    y = curr_state.position[1]
    w = obs_shape.width
    h = obs_shape.length
    # add pi/2 because CommonRoad 'neutral' position is horizontal
    # therefore 'width' is actually length and 'length' is actually width
    # alternative: w = length, h = width
    o = curr_state.orientation + np.pi / 2
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
    return shp.Polygon([top_right, top_left, bottom_left, bottom_right])


def add_safety_buffer(obs, curr_step: int, reverse=False):
    # increase size of car by distance travelled inbetween steps
    buffer = obs.state_at_time(curr_step).velocity * 0.1
    # create new obstacles with updated shapes
    if reverse is False:
        new_obs_shape = Rectangle(obs.obstacle_shape.length + buffer, obs.obstacle_shape.width)
    else:
        new_obs_shape = Rectangle(obs.obstacle_shape.length - buffer, obs.obstacle_shape.width)
    if type(obs) is DynamicObstacle:
        new_obs_prediction = TrajectoryPrediction(obs.prediction.trajectory, new_obs_shape)
        new_obs = DynamicObstacle(obs.obstacle_id, obs.obstacle_type, new_obs_shape, obs.initial_state, new_obs_prediction)
    else:
        new_obs = StaticObstacle(obs.obstacle_id, obs.obstacle_type, new_obs_shape, obs.initial_state)
    return new_obs


def convert_multi_to_linestring(geometry: shp.base.BaseGeometry):
    if isinstance(geometry, shp.MultiLineString):
        multi_coordinates = []
        for line in geometry:
            for c in line.coords:
                multi_coordinates.append(shp.Point(c))
        return shp.LineString(multi_coordinates)
    else:
        return geometry


def check_predictability(curr_state: State, curr_lanelet: Lanelet, obs_shape: Rectangle):
    # case 1: return False if obstacle is crossing left or right vertices
    obs = compute_shape(curr_state, obs_shape)
    left_vertices = curr_lanelet.left_vertices
    right_vertices = curr_lanelet.right_vertices
    if obs.intersects(shp.LineString(left_vertices)) or obs.intersects(shp.LineString(right_vertices)):
        return False
    # case 2: return False if obstacle orientation and center vertices orientation are too far off
    center_vertices = curr_lanelet.center_vertices
    # create reference_path from center_vertices
    reference_path = shp.LineString(center_vertices)
    # compute vertices orientation at closest point to obstacle
    curr_point = shp.Point(curr_state.position[0], curr_state.position[1])
    v1 = reference_path.interpolate(reference_path.project(curr_point))
    v2 = reference_path.interpolate(reference_path.project(curr_point)+0.1)
    v_orientation = compute_orientation([v1.x, v1.y], [v2.x, v2.y])
    o_orientation = curr_state.orientation
    # orientation difference
    orientation_difference = np.arctan2(np.sin(v_orientation - o_orientation), np.cos(v_orientation - o_orientation))
    if abs(orientation_difference) > np.deg2rad(5.0):
        return False
    return True


def find_ref_path(scenario: Scenario, curr_state: State):
    # find lanelet and center_vertices
    if not scenario.lanelet_network.find_lanelet_by_position([curr_state.position])[0]:
        return None, None
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


def compute_position(scenario: Scenario, curr_state: State, time_step_size):
    curr_point, parallel_ref_path = find_ref_path(scenario, curr_state)
    if curr_point is None and parallel_ref_path is None:
        return None
    # move along parallel_ref_path by target_distance
    target_distance = time_step_size * curr_state.velocity
    new_position = parallel_ref_path.interpolate(parallel_ref_path.project(curr_point) + target_distance)
    return np.array([new_position.x, new_position.y])


def predict_trajectory(obs: DynamicObstacle, scenario: Scenario, current_count: int):
    # compute current visible states
    visible_states = [obs.initial_state]
    if current_count > 0:
        visible_states.extend(obs.prediction.trajectory.state_list[:current_count])
    current_state = visible_states[-1]
    # find lanelet and center_vertices
    lanelet_id = scenario.lanelet_network.find_lanelet_by_position([current_state.position])
    if lanelet_id[0]:
        curr_lanelet = scenario.lanelet_network.find_lanelet_by_id(lanelet_id[0][0])
        # predictable check on center vertices
        if len(scenario.lanelet_network.intersections) == 0:
            predictability = check_predictability(current_state, curr_lanelet, obs.obstacle_shape)
        else:
            predictability = False
    else:
        predictability = False
    prediction_range = 20
    for i in range(0, prediction_range):
        if predictability is True:
            # predict next states along center vertices
            tmp_position = compute_position(scenario, visible_states[-1], 0.1)
            # stop prediction if the end of the current lanelet is reached or the position is off the lanelet network
            if euclidian_distance(tmp_position, visible_states[-1].position) < visible_states[-1].velocity * 0.1 or tmp_position is None:
                predictability = False
                tmp_position_x = visible_states[-1].position[0] + np.cos(visible_states[-1].orientation) * visible_states[-1].velocity * 0.1
                tmp_position_y = visible_states[-1].position[1] + np.sin(visible_states[-1].orientation) * visible_states[-1].velocity * 0.1
                tmp_position = np.array([tmp_position_x, tmp_position_y])
                tmp_state = State(position=tmp_position,
                                  velocity=visible_states[-1].velocity,
                                  orientation=visible_states[-1].orientation,
                                  time_step=visible_states[-1].time_step + 1)
                visible_states.append(tmp_state)
                continue
            tmp_orientation = compute_orientation([visible_states[-1].position[0], visible_states[-1].position[1]], [tmp_position[0], tmp_position[1]])
            tmp_state = State(position=tmp_position,
                              velocity=visible_states[-1].velocity,
                              orientation=tmp_orientation,
                              time_step=visible_states[-1].time_step + 1)
            visible_states.append(tmp_state)
        else:
            # predict next states in straight direction
            tmp_position_x = visible_states[-1].position[0] + np.cos(visible_states[-1].orientation) * visible_states[-1].velocity * 0.1
            tmp_position_y = visible_states[-1].position[1] + np.sin(visible_states[-1].orientation) * visible_states[-1].velocity * 0.1
            tmp_position = np.array([tmp_position_x, tmp_position_y])
            tmp_state = State(position=tmp_position,
                              velocity=visible_states[-1].velocity,
                              orientation=visible_states[-1].orientation,
                              time_step=visible_states[-1].time_step + 1)
            visible_states.append(tmp_state)
    # create new obstacle with predicted trajectory
    new_obs_trajectory = Trajectory(1, visible_states[1:])
    new_obs_prediction = TrajectoryPrediction(new_obs_trajectory, obs.obstacle_shape)
    new_obs = DynamicObstacle(obs.obstacle_id, obs.obstacle_type, obs.obstacle_shape, obs.initial_state, new_obs_prediction)
    return new_obs


def vision_radius(crfr: CommonRoadFileReader, current_count: int, ego_vehicle: DynamicObstacle, road_boundary_sg_triangles, true_vision: bool, safety_buffer: bool):
    scenario, problem_set = crfr.open()
    # add safety buffer to obstacles to avoid crashes inbetween frames
    if safety_buffer is True:
        for obs in scenario.dynamic_obstacles:
            if current_count <= len(obs.prediction.trajectory.state_list):
                new_obs = add_safety_buffer(obs, current_count)
                scenario.remove_obstacle(obs)
                scenario.add_objects(new_obs)
        for obs in scenario.static_obstacles:
            new_obs = add_safety_buffer(obs, current_count)
            scenario.remove_obstacle(obs)
            scenario.add_objects(new_obs)
    if true_vision is True:
        # ego position
        attention_distance = 5.0
        if ego_vehicle is not None:
            ego_x = ego_vehicle.state_at_time(current_count).position[0]
            ego_y = ego_vehicle.state_at_time(current_count).position[1]
            vision_distance = ego_vehicle.state_at_time(current_count).velocity * attention_distance
        else:
            ego_x = list(problem_set.planning_problem_dict.values())[0].initial_state.position[0]
            ego_y = list(problem_set.planning_problem_dict.values())[0].initial_state.position[1]
            vision_distance = list(problem_set.planning_problem_dict.values())[0].initial_state.velocity * attention_distance
        obstacles_out_of_vision = []
        for obs in scenario.dynamic_obstacles:
            if len(obs.prediction.trajectory.state_list) >= current_count:
                # obstacle position
                obs_x = obs.state_at_time(current_count).position[0]
                obs_y = obs.state_at_time(current_count).position[1]
                # distance ego to obstacle
                distance = euclidian_distance([obs_x, obs_y], [ego_x, ego_y])
                # remove obstacles if they are out of the vision radius
                if distance > vision_distance:
                    obstacles_out_of_vision.append(obs)
                    scenario.remove_obstacle(obs)
                # predict trajectory while assuming straight driving direction
                else:
                    new_obs = predict_trajectory(obs, scenario, current_count)
                    # remove old obstacle and add new obstacle
                    scenario.remove_obstacle(obs)
                    scenario.add_objects(new_obs)
        for obs in scenario.static_obstacles:
            # obstacle position
            obs_x = obs.state_at_time(current_count).position[0]
            obs_y = obs.state_at_time(current_count).position[1]
            # distance ego to obstacle
            distance = np.sqrt(np.power((obs_x - ego_x), 2) + np.power((obs_y - ego_y), 2))
            # remove obstacles if they are out of the vision radius
            if distance > vision_distance:
                obstacles_out_of_vision.append(obs)
                scenario.remove_obstacle(obs)
        # create new collision checker
        collision_checker_scenario = create_collision_checker(scenario)
        collision_checker_scenario.add_collision_object(road_boundary_sg_triangles)
        # add obstacles to scenario again for correct visualization
        for obs in obstacles_out_of_vision:
            scenario.add_objects(obs)
    else:
        # create new collision checker
        collision_checker_scenario = create_collision_checker(scenario)
        collision_checker_scenario.add_collision_object(road_boundary_sg_triangles)
    # reverse safety buffer for correct visualization
    if safety_buffer is True:
        for obs in scenario.dynamic_obstacles:
            if current_count <= len(obs.prediction.trajectory.state_list):
                new_obs = add_safety_buffer(obs, current_count, reverse=True)
                scenario.remove_obstacle(obs)
                scenario.add_objects(new_obs)
        for obs in scenario.static_obstacles:
            new_obs = add_safety_buffer(obs, current_count, reverse=True)
            scenario.remove_obstacle(obs)
            scenario.add_objects(new_obs)
    return scenario, collision_checker_scenario
