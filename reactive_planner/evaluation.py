import json
import os
from glob import glob
import matplotlib.pyplot as plt
import numpy as np

from jmetal.util.solution import get_non_dominated_solutions, print_variables_to_file, print_function_values_to_file
from jmetal.core.solution import FloatSolution
from jmetal.core.quality_indicator import HyperVolume
from avoidability import is_collision
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType, Rectangle, State
from commonroad.common.file_reader import CommonRoadFileReader


def compute_collision_timestep(file_id, reconstruction_data):
    files = sorted(glob(f"./example_scenarios/{file_id}.xml"))
    crfr = CommonRoadFileReader(files[0])
    scenario, _ = crfr.open()
    if scenario.obstacles:
        obs_dimensions = [scenario.obstacles[0].obstacle_shape.length, scenario.obstacles[0].obstacle_shape.width]
        ego_dimensions = [4.508, 1.61]
        ego_positions = reconstruction_data['ego_positions']
        ego_orientations = reconstruction_data['ego_orientations']
        obs_positions = reconstruction_data['obs_positions']
        obs_orientations = reconstruction_data['obs_orientations']
        assert len(ego_positions) == len(ego_orientations) and len(obs_positions) == len(obs_orientations)
        tmp_counter = 0
        for i in range(min(len(ego_positions), len(obs_positions))):
            tmp_ego_state = State(position=ego_positions[i], orientation=ego_orientations[i], time_step=0)
            tmp_obs_state = State(position=obs_positions[i], orientation=obs_orientations[i], time_step=0)
            tmp_ego = DynamicObstacle(41, ObstacleType.CAR, Rectangle(obs_dimensions[0], obs_dimensions[1]), tmp_ego_state)
            tmp_obs = DynamicObstacle(42, ObstacleType.CAR, Rectangle(ego_dimensions[0], ego_dimensions[1]), tmp_obs_state)
            if is_collision(tmp_ego, tmp_obs, 0) is False:
                tmp_counter += 1
            else:
                return tmp_counter
        else:
            return -1
    else:
        return -1


def extract_solutions(directory: str):
    solutions = []
    for f in glob(directory + '/*.json'):
        file_id = f[(f.rfind('/') + 1):-4]
        if file_id[0] == 't':
            with open(f, 'r') as curr_file:
                curr_content = json.load(curr_file)
                for key in curr_content:
                    curr_gen = curr_content[key]['attributes']['generation_number']
                    curr_ind = curr_content[key]['attributes']['individual_number']
                    crash = curr_content[key]['attributes']['crash']
                    goal_reached = curr_content[key]['attributes']['goal_reached']
                    extension_time = curr_content[key]['objectives']['extension_time']
                    danger_difference = curr_content[key]['objectives']['danger_difference']
                    parameter_difference = curr_content[key]['objectives']['parameter_difference']
                    # create new solution (no variables, no lower, no upper)
                    new_solution = FloatSolution(lower_bound=[], upper_bound=[], number_of_objectives=3)
                    new_solution.variables = [extension_time, key]
                    new_solution.objectives[0] = extension_time
                    new_solution.objectives[1] = danger_difference
                    new_solution.objectives[2] = parameter_difference
                    new_solution.attributes['generation_number'] = curr_gen
                    new_solution.attributes['individual_number'] = curr_ind
                    new_solution.attributes['crash'] = crash
                    new_solution.attributes['goal_reached'] = goal_reached
                    solutions.append(new_solution)
    return solutions


def extract_generation(solutions: [], generation_number: int, acc=False):
    generation = []
    for s in solutions:
        if acc is True:
            if s.attributes['generation_number'] <= generation_number:
                generation.append(s)
        else:
            if s.attributes['generation_number'] == generation_number:
                generation.append(s)
    return generation


def compute_reference_point(solutions: []):
    max_t = 0
    max_d = 0
    max_p = 0
    for s in solutions:
        max_t = max(max_t, s.objectives[0])
        max_d = max(max_d, s.objectives[1])
        max_p = max(max_p, s.objectives[2])
    return [max_t, max_d, max_p]


def plot_hv(hv: [], filepath: str, title: str):
    fig = plt.figure()
    fig.suptitle(title, fontsize=16)
    hv_count = np.arange(len(hv))
    plt.xticks(hv_count[::2])
    plt.plot(hv_count, hv)
    plt.savefig(filepath + '.png', format='png', dpi=1000)
    plt.close(fig=fig)


def compute_hv(solutions: [], filepath: str):
    # compute reference point
    ref_point = compute_reference_point(solutions)
    hv = HyperVolume(reference_point=ref_point)
    # hyper volume for each individual generation
    ghv = []
    for i in range(20):
        igen = get_non_dominated_solutions(extract_generation(solutions, i))
        igen_obj = []
        for k in igen:
            igen_obj.append([k.objectives[0], k.objectives[1], k.objectives[2]])
        ghv.append(hv.compute(igen_obj))
    plot_hv(ghv, f'{filepath}_individual', 'Individual Hypervolume')
    # accumulated hyper volume
    ahv = []
    for i in range(20):
        agen = get_non_dominated_solutions(extract_generation(solutions, i, acc=True))
        agen_obj = []
        for k in agen:
            agen_obj.append([k.objectives[0], k.objectives[1], k.objectives[2]])
        ahv.append(hv.compute(agen_obj))
    plot_hv(ahv, f'{filepath}_accumulated', 'Accumulated Hypervolume')


def search_progress(solutions: [], filename: str):
    extension_time = []
    danger_difference = []
    weight_difference = []
    solution_count = np.arange(len(solutions))
    for s in solutions:
        extension_time.append(s.objectives[0])
        danger_difference.append(s.objectives[1])
        weight_difference.append(s.objectives[2])
    ax1 = plt.subplot(311)
    plt.plot(solution_count, extension_time)
    ax1.set_ylabel('extension_time')
    ax2 = plt.subplot(312, sharex=ax1)
    plt.plot(solution_count, danger_difference)
    ax2.set_ylabel('danger_difference')
    ax3 = plt.subplot(313, sharex=ax1)
    plt.plot(solution_count, weight_difference)
    ax3.set_ylabel('weight_difference')
    plt.savefig(filename + '.png', format='png', dpi=1000)
    plt.close()


def custom_plot(solutions: [], filename: str, ref_result: {}, title='Custom Plot', show=False, save_angles=False):
    fig = plt.figure()
    fig.suptitle(title, fontsize=16)
    ax = fig.add_subplot(projection='3d')
    ax.scatter([s.objectives[0] for s in solutions],
               [s.objectives[1] for s in solutions],
               [s.objectives[2] for s in solutions],
               color=[('red' if s.attributes['crash'] is True else ('blue' if s.attributes['goal_reached'] is True else 'purple')) for s in solutions])
    # add surfaces to visualize lower and upper bound
    max_danger_change = min_danger_change = 0.0
    max_weight_change = min_weight_change = 0.0
    for s in solutions:
        if s.objectives[1] > max_danger_change:
            max_danger_change = s.objectives[1]
        if s.objectives[1] < min_danger_change:
            min_danger_change = s.objectives[1]
        if s.objectives[2] > max_weight_change:
            max_weight_change = s.objectives[2]
        if s.objectives[2] < min_weight_change:
            min_weight_change = s.objectives[2]
    ys = np.linspace(min_danger_change*1.1, max_danger_change*1.1, 100)
    zs = np.linspace(min_weight_change*1.1, max_weight_change*1.1, 100)
    y, z = np.meshgrid(ys, zs)
    if ref_result['upper_bound'] == -1000.0:
        ax.plot_surface(ref_result['lower_bound'], y, z, alpha=0.5, color='red')
        boundary_info = f'lb: {ref_result["lower_bound"]}, ub: /'
    else:
        if ref_result['lower_bound'] != ref_result['upper_bound']:
            ax.plot_surface(ref_result['lower_bound'], y, z, alpha=0.5, color='red')
            ax.plot_surface(ref_result['upper_bound'], y, z, alpha=0.5, color='green')
        else:
            ax.plot_surface(ref_result['lower_bound'], y, z, alpha=0.5, color='yellow')
        boundary_info = f'lb: {ref_result["lower_bound"]}, ub: {ref_result["upper_bound"]}'
    ax.set_title(f'ref_result: ({ref_result["max_danger"]}, {ref_result["crash"]}), '
                 f'length: {ref_result["scenario_length"]}/{ref_result["collision_timestep"]}, '
                 f'{boundary_info}')
    ax.set_xlabel('extension_time')
    ax.set_ylabel('danger_difference')
    ax.set_zlabel('parameter_difference')
    ax.relim()
    ax.autoscale_view(True, True, True)
    ax.locator_params(nbins=4)
    if save_angles is True:
        angles = [0.0, 30.0, 60.0, 90.0]
        for a in angles:
            ax.view_init(elev=30.0, azim=a)
            plt.savefig(filename + str(a) + '.png', format='png', dpi=1000)
    else:
        ax.view_init(elev=30.0, azim=30.0)
        if show is True:
            plt.show()
        else:
            plt.savefig(filename + '.png', format='png', dpi=1000)
    plt.close(fig=fig)


def evaluate_result(filename: str, root_dir: str):
    # extract solutions from NSGA-II results
    file_id = filename[(filename.rfind('/') + 1):-4]
    information_directory = f'./{root_dir}/{file_id}'
    solutions = extract_solutions(information_directory)
    print(f'total number of solutions: {len(solutions)}')

    # clear everything but the search results
    for f in os.scandir(information_directory):
        if not f.name.endswith('.json'):
            os.unlink(f)

    # extract reference result
    ref_file_path = f'{information_directory}/{file_id}.json'
    with open(ref_file_path, 'r') as rf:
        content = json.load(rf)
        for k in content:
            ref_attributes = content[k]['attributes']
            collision_timestep = compute_collision_timestep(file_id, content[k])
    ref_result = {'max_danger': ref_attributes['max_danger'], 'crash': ref_attributes['crash'],
                  'goal_reached': ref_attributes['goal_reached'], 'scenario_length': ref_attributes['scenario_length'],
                  'lower_bound': ref_attributes['lower_bound'], 'upper_bound': ref_attributes['upper_bound'],
                  'collision_timestep': collision_timestep}

    # visualize pareto front
    pareto = get_non_dominated_solutions(solutions)
    print_function_values_to_file(pareto, f'{information_directory}/function_values_pareto')
    print_variables_to_file(pareto, f'{information_directory}/variables_pareto')
    custom_plot(pareto, f'{information_directory}/pareto_front', ref_result, 'Pareto Front')

    # visualize all solutions
    print_function_values_to_file(solutions, f'{information_directory}/function_values')
    print_variables_to_file(solutions, f'{information_directory}/variables')
    custom_plot(solutions, f'{information_directory}/solutions', ref_result, 'Solutions')

    # visualize search progress
    search_progress(sorted(solutions, key=lambda s: s.attributes['individual_number']), f'{information_directory}/progress')

    # visualize hypervolume
    compute_hv(solutions, f'{information_directory}/hv')


def interactive_plot(filename: str, root_dir: str):
    # extract solutions from NSGA-II results
    file_id = filename[(filename.rfind('/') + 1):-4]
    information_directory = f'./{root_dir}/{file_id}'
    solutions = extract_solutions(information_directory)

    # extract reference result
    ref_file_path = f'{information_directory}/{file_id}.json'
    with open(ref_file_path, 'r') as rf:
        content = json.load(rf)
        for k in content:
            ref_attributes = content[k]['attributes']
            collision_timestep = compute_collision_timestep(file_id, content[k])
    ref_result = {'max_danger': ref_attributes['max_danger'], 'crash': ref_attributes['crash'],
                  'goal_reached': ref_attributes['goal_reached'], 'scenario_length': ref_attributes['scenario_length'],
                  'lower_bound': ref_attributes['lower_bound'], 'upper_bound': ref_attributes['upper_bound'],
                  'collision_timestep': collision_timestep}

    # visualize solutions
    custom_plot(solutions, f'{information_directory}/solutions', ref_result, 'Solutions', show=True)


if __name__ == "__main__":
    scenario_files = ['./example_scenarios/CD_M_Static.xml',
                      # original CRISCE scenarios
                      './example_scenarios/CRISCE_99817.xml',
                      './example_scenarios/CRISCE_105222.xml',
                      './example_scenarios/CRISCE_120305.xml',
                      './example_scenarios/CRISCE_122168.xml',
                      './example_scenarios/CRISCE_156722.xml',
                      './example_scenarios/CRISCE_171831.xml',
                      # adjusted CRISCE scenarios
                      './example_scenarios/CRISCE_M_99817.xml',
                      './example_scenarios/CRISCE_M_105222.xml',
                      './example_scenarios/CRISCE_M_105222_I1.xml',
                      './example_scenarios/CRISCE_M_105222_I2.xml',
                      './example_scenarios/CRISCE_M_105222_I3.xml',
                      './example_scenarios/CRISCE_M_105222_I4.xml',
                      './example_scenarios/CRISCE_M_122168_1.xml',
                      './example_scenarios/CRISCE_M_122168_2.xml',
                      './example_scenarios/CRISCE_M_156722.xml',
                      # adjusted TUM CommonRoad scenarios
                      './example_scenarios/CR_USA_Peach.xml',
                      './example_scenarios/CR_ZAM_Moelln.xml',
                      './example_scenarios/CR_ZAM_Tjunction_1.xml',
                      './example_scenarios/CR_ZAM_Tjunction_2.xml',
                      './example_scenarios/CR_ZAM_Zaventem.xml']

    interactive = False

    if interactive:
        for sf in scenario_files:
            print(sf)
            interactive_plot(sf, root_dir='information_files')
    else:
        for sf in scenario_files:
            evaluate_result(sf, root_dir='information_files')

