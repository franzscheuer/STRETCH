import glob
import os
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.planning.planning_problem import PlanningProblemSet

from lanelet_extension import extend_lanelets, rotate_lanelets, extend_all_lanelets
from trajectory_extension import extend_planning_problem, extend_obstacle, reduce_planning_problem, reduce_obstacle, \
    change_velocity, compute_distance_to_end, rotate_state, rotate_goal_state


def compute_scenario_center(lanelet_network: LaneletNetwork):
    lx = lanelet_network.lanelets[0].left_vertices[0][0]
    hx = lanelet_network.lanelets[0].left_vertices[0][0]
    ly = lanelet_network.lanelets[0].left_vertices[0][1]
    hy = lanelet_network.lanelets[0].left_vertices[0][1]
    for lanelet in lanelet_network.lanelets:
        for c in lanelet.left_vertices:
            tmp_x = c[0]
            tmp_y = c[1]
            lx = tmp_x if tmp_x < lx else lx
            hx = tmp_x if tmp_x > hx else hx
            ly = tmp_y if tmp_y < ly else ly
            hy = tmp_y if tmp_y > hy else hy
    return (lx + hx)/2, (ly + hy)/2


def rotate_scenario(scenario_filename: str, output_filename: str, scenario_rotation: float, plot=False):
    # load scenario and planning problem
    files = sorted(glob.glob(scenario_filename))
    crfr = CommonRoadFileReader(files[0])
    scenario, problem_set = crfr.open()
    planning_problem = list(problem_set.planning_problem_dict.values())[0]

    # create new scenario
    new_scenario = Scenario(dt=scenario.dt, scenario_id=scenario.scenario_id, author=scenario.author,
                            tags=scenario.tags, affiliation=scenario.affiliation, source=scenario.source,
                            location=scenario.location)

    cx, cy = compute_scenario_center(scenario.lanelet_network)

    # rotate lanelets
    new_scenario.lanelet_network = rotate_lanelets(scenario.lanelet_network, scenario_rotation, cx, cy)

    # rotate planning problem
    new_planning_problem = planning_problem
    new_planning_problem.initial_state = rotate_state(planning_problem.initial_state, scenario_rotation, cx, cy)
    new_planning_problem.goal.state_list[0] = rotate_goal_state(planning_problem.goal.state_list[0], scenario_rotation, cx, cy)

    # rotate dynamic obstacles
    for obs in scenario.dynamic_obstacles:
        obs.initial_state = rotate_state(obs.initial_state, scenario_rotation, cx, cy)
        for i, s in enumerate(obs.prediction.trajectory.state_list):
            obs.prediction.trajectory.state_list[i] = rotate_state(s, scenario_rotation, cx, cy)
        new_scenario.add_objects(obs)

    # rotate static obstacles
    for obs in scenario.static_obstacles:
        obs.initial_state = rotate_state(obs.initial_state, scenario_rotation, cx, cy)
        new_scenario.add_objects(obs)

    if plot is True:
        rnd = MPRenderer()
        new_scenario.draw(rnd)
        new_planning_problem.draw(rnd)
        rnd.render(show=True)

    # write to file
    fw = CommonRoadFileWriter(new_scenario, PlanningProblemSet([new_planning_problem]))
    os.makedirs(os.path.dirname(output_filename), exist_ok=True)
    fw.write_to_file(output_filename, OverwriteExistingFile.ALWAYS)


def extend_lanelet_network(scenario_filename: str, output_filename: str, extension_time: float, plot=False):
    # load scenario and planning problem
    files = sorted(glob.glob(scenario_filename))
    crfr = CommonRoadFileReader(files[0])
    scenario, problem_set = crfr.open()
    planning_problem = list(problem_set.planning_problem_dict.values())[0]

    # create new scenario and compute extended lanelet network
    new_scenario = Scenario(dt=scenario.dt, scenario_id=scenario.scenario_id, author=scenario.author,
                            tags=scenario.tags, affiliation=scenario.affiliation, source=scenario.source,
                            location=scenario.location)

    # extend lanelet network by bigger distance
    planning_problem_extension_distance = planning_problem.initial_state.velocity * extension_time
    if not scenario.dynamic_obstacles:
        obstacle_extension_distance = 0.0
    else:
        obstacle_extension_distance = scenario.dynamic_obstacles[0].initial_state.velocity * extension_time
    extension_distance = max(planning_problem_extension_distance, obstacle_extension_distance)

    # extend lanelet network
    new_scenario.lanelet_network = extend_all_lanelets(scenario.lanelet_network, extension_distance)

    # add obstacles
    for obs in scenario.dynamic_obstacles:
        new_scenario.add_objects(obs)
    for obs in scenario.static_obstacles:
        new_scenario.add_objects(obs)

    if plot is True:
        rnd = MPRenderer()
        new_scenario.draw(rnd)
        planning_problem.draw(rnd)
        rnd.render(show=True)

    # write to file
    fw = CommonRoadFileWriter(new_scenario, PlanningProblemSet([planning_problem]))
    os.makedirs(os.path.dirname(output_filename), exist_ok=True)
    fw.write_to_file(output_filename, OverwriteExistingFile.ALWAYS)


def extend_scenario(scenario_filename: str, output_filename: str, extension_time: float,
                    planning_problem_extension_radius: float, obstacle_extension_radius: float, plot=False):
    # load scenario and planning problem
    files = sorted(glob.glob(scenario_filename))
    crfr = CommonRoadFileReader(files[0])
    scenario, problem_set = crfr.open()
    planning_problem = list(problem_set.planning_problem_dict.values())[0]

    # create new scenario and compute extended lanelet network
    new_scenario = Scenario(dt=scenario.dt, scenario_id=scenario.scenario_id, author=scenario.author,
                            tags=scenario.tags, affiliation=scenario.affiliation, source=scenario.source,
                            location=scenario.location)

    # find relevant planning problem lanelets
    initial_id = scenario.lanelet_network.find_lanelet_by_position([planning_problem.initial_state.position])[0]
    initial_lanelets = [scenario.lanelet_network.find_lanelet_by_id(initial_id[0])]
    # include adjacent lanelets with same direction
    adjacent_lanelets = []
    for i in initial_lanelets:
        curr_lanelet = i
        while curr_lanelet.adj_left_same_direction:
            adj_lanelet = scenario.lanelet_network.find_lanelet_by_id(curr_lanelet.adj_left)
            adjacent_lanelets.append(adj_lanelet)
            curr_lanelet = adj_lanelet
        curr_lanelet = i
        while curr_lanelet.adj_right_same_direction:
            adj_lanelet = scenario.lanelet_network.find_lanelet_by_id(curr_lanelet.adj_right)
            adjacent_lanelets.append(adj_lanelet)
            curr_lanelet = adj_lanelet
    # combine initial_lanelets and adjacent lanelets and remove duplicates
    planning_problem_lanelet_ids = []
    for i in initial_lanelets:
        included = False
        for k in planning_problem_lanelet_ids:
            if i.lanelet_id == k:
                included = True
                break
        if included is False:
            planning_problem_lanelet_ids.append(i.lanelet_id)
    for i in adjacent_lanelets:
        included = False
        for k in planning_problem_lanelet_ids:
            if i.lanelet_id == k:
                included = True
                break
        if included is False:
            planning_problem_lanelet_ids.append(i.lanelet_id)

    # find relevant obstacle lanelets
    obs_lanelets = []
    for obs in scenario.dynamic_obstacles:
        obs_id = scenario.lanelet_network.find_lanelet_by_position([obs.initial_state.position])[0]
        obs_lanelets.append(scenario.lanelet_network.find_lanelet_by_id(obs_id[0]))
    # include adjacent lanelets with same direction
    adjacent_lanelets = []
    for i in obs_lanelets:
        curr_lanelet = i
        while curr_lanelet.adj_left_same_direction:
            adj_lanelet = scenario.lanelet_network.find_lanelet_by_id(curr_lanelet.adj_left)
            adjacent_lanelets.append(adj_lanelet)
            curr_lanelet = adj_lanelet
        curr_lanelet = i
        while curr_lanelet.adj_right_same_direction:
            adj_lanelet = scenario.lanelet_network.find_lanelet_by_id(curr_lanelet.adj_right)
            adjacent_lanelets.append(adj_lanelet)
            curr_lanelet = adj_lanelet
    # combine obs_lanelets and adjacent lanelets and remove duplicates
    obstacle_lanelet_ids = []
    for i in obs_lanelets:
        included = False
        for k in obstacle_lanelet_ids:
            if i.lanelet_id == k:
                included = True
                break
        if included is False:
            obstacle_lanelet_ids.append(i.lanelet_id)
    for i in adjacent_lanelets:
        included = False
        for k in obstacle_lanelet_ids:
            if i.lanelet_id == k:
                included = True
                break
        if included is False:
            obstacle_lanelet_ids.append(i.lanelet_id)

    # extend scenario by extension_time
    planning_problem_extension_distance = planning_problem.initial_state.velocity * extension_time
    if not scenario.dynamic_obstacles:
        obstacle_extension_distance = 0.0
    else:
        obstacle_extension_distance = scenario.dynamic_obstacles[0].initial_state.velocity * extension_time

    # extend planning problem lanelets
    new_scenario.lanelet_network = extend_lanelets(scenario.lanelet_network, planning_problem_lanelet_ids,
                                                   extension_distance=planning_problem_extension_distance,
                                                   extension_radius=planning_problem_extension_radius)

    # extend obstacle lanelets
    new_scenario.lanelet_network = extend_lanelets(new_scenario.lanelet_network, obstacle_lanelet_ids,
                                                   extension_distance=obstacle_extension_distance,
                                                   extension_radius=obstacle_extension_radius)

    # extend planning problem
    new_planning_problem = extend_planning_problem(new_scenario, planning_problem, extension_time)

    # extend obstacle trajectories
    for obs in scenario.dynamic_obstacles:
        new_obstacle = extend_obstacle(new_scenario, obs, extension_time)
        new_scenario.add_objects(new_obstacle)

    for obs in scenario.static_obstacles:
        new_scenario.add_objects(obs)

    if plot is True:
        rnd = MPRenderer()
        new_scenario.draw(rnd)
        new_planning_problem.draw(rnd)
        rnd.render(show=True)

    # write to file
    fw = CommonRoadFileWriter(new_scenario, PlanningProblemSet([new_planning_problem]))
    os.makedirs(os.path.dirname(output_filename), exist_ok=True)
    fw.write_to_file(output_filename, OverwriteExistingFile.ALWAYS)


def reduce_scenario(scenario_filename: str, output_filename: str, reduction_time: float, plot=False):
    # load scenario and planning problem
    files = sorted(glob.glob(scenario_filename))
    crfr = CommonRoadFileReader(files[0])
    scenario, problem_set = crfr.open()
    planning_problem = list(problem_set.planning_problem_dict.values())[0]

    # create new scenario
    new_scenario = Scenario(dt=scenario.dt, scenario_id=scenario.scenario_id, author=scenario.author,
                            tags=scenario.tags, affiliation=scenario.affiliation, source=scenario.source,
                            location=scenario.location)

    # copy lanelets
    new_scenario.lanelet_network = scenario.lanelet_network

    # reduce planning problem
    new_planning_problem = reduce_planning_problem(new_scenario, planning_problem, reduction_time)

    # reduce obstacle trajectories
    for obs in scenario.dynamic_obstacles:
        new_obstacle = reduce_obstacle(obs, reduction_time)
        if new_obstacle:
            new_scenario.add_objects(new_obstacle)

    for obs in scenario.static_obstacles:
        new_scenario.add_objects(obs)

    if plot is True:
        rnd = MPRenderer()
        new_scenario.draw(rnd)
        new_planning_problem.draw(rnd)
        rnd.render(show=True)

    # write to file
    fw = CommonRoadFileWriter(new_scenario, PlanningProblemSet([new_planning_problem]))
    os.makedirs(os.path.dirname(output_filename), exist_ok=True)
    fw.write_to_file(output_filename, OverwriteExistingFile.ALWAYS)


def change_velocities(scenario_filename: str, output_filename: str, change_factor=0.0, obs_velocity=0.0, ego_velocity=0.0, plot=False):
    # load scenario and planning problem
    files = sorted(glob.glob(scenario_filename))
    crfr = CommonRoadFileReader(files[0])
    scenario, problem_set = crfr.open()
    planning_problem = list(problem_set.planning_problem_dict.values())[0]

    # create new scenario
    new_scenario = Scenario(dt=scenario.dt, scenario_id=scenario.scenario_id, author=scenario.author,
                            tags=scenario.tags, affiliation=scenario.affiliation, source=scenario.source,
                            location=scenario.location)

    # copy lanelets
    new_scenario.lanelet_network = scenario.lanelet_network

    # update planning problem velocity
    if change_factor > 0.0 and ego_velocity > 0.0:
        raise ValueError('you cannot speed up the scenario and change ego_velocity at the same time')
    if change_factor > 0.0:
        planning_problem.initial_state.velocity *= change_factor
    if ego_velocity > 0.0:
        planning_problem.initial_state.velocity = ego_velocity

    # update obstacle velocity
    if change_factor > 0.0 and obs_velocity > 0.0:
        raise ValueError('you cannot speed up the scenario and change obs_velocity at the same time')
    for obs in scenario.dynamic_obstacles:
        if obs_velocity > 0.0:
            new_obstacle = change_velocity(obs, obs_velocity)
            new_scenario.add_objects(new_obstacle)
        if change_factor > 0.0:
            new_obstacle = change_velocity(obs, change_factor * obs.initial_state.velocity)
            new_scenario.add_objects(new_obstacle)

    for obs in scenario.static_obstacles:
        new_scenario.add_objects(obs)

    if plot is True:
        rnd = MPRenderer()
        new_scenario.draw(rnd)
        planning_problem.draw(rnd)
        rnd.render(show=True)

    # write to file
    fw = CommonRoadFileWriter(new_scenario, PlanningProblemSet([planning_problem]))
    fw.write_to_file(output_filename, OverwriteExistingFile.ALWAYS)


def reduction_is_possible(scenario_filename: str, reduction_time: float):
    # load scenario and planning problem
    files = sorted(glob.glob(scenario_filename))
    crfr = CommonRoadFileReader(files[0])
    scenario, problem_set = crfr.open()
    planning_problem = list(problem_set.planning_problem_dict.values())[0]
    # desired and available reduction
    desired_reduction = 0.1 * planning_problem.initial_state.velocity * int(reduction_time * 10)
    available_reduction = compute_distance_to_end(scenario, planning_problem)
    return True if desired_reduction < available_reduction else False


if __name__ == "__main__":
    scenario_file = './example_scenarios/ZAM_INTERSECTION1-1_1_T-1.xml'
    time = [1.0, 2.0, 3.0, 4.0, 5.0]
    radius = [0.0]
    for t in time:
        for pla_r in radius:
            for obs_r in radius:
                extension_file = f'./example_scenarios/extensions/t_{t}_pla_r_{pla_r}_obs_r_{obs_r}.xml'
                extend_scenario(scenario_file, extension_file, t, pla_r, obs_r, plot=True)

    # scenario reduction
    # time = [1.0]
    # for t in time:
    #     if reduction_is_possible(scenario_file, t):
    #         reduction_file = f'./example_scenarios/reductions/t_{t}.xml'
    #         reduce_scenario(scenario_file, reduction_file, t, plot=True)

    # scenario velocity alteration
    # velocity_file = f'{scenario_file[:-4]}-V.xml'
    # change_velocities(scenario_file, velocity_file, change_factor=1.5)
    # change_velocities(scenario_file, velocity_file, obs_velocity=50.0, ego_velocity=10.0)

    # scenario rotation
    # rotation = [0.0, 0.7853, 1.5708, 2.3562, 3.1416]
    # for r in rotation:
    #     rotation_file = f'{scenario_file[:-4]}-R.xml'
    #     rotate_scenario(scenario_file, rotation_file, r, plot=True)

    # lanelet network extension
    # extend_lanelet_network(scenario_file, extension_file, 0.5, plot=True)
