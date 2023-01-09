import crash_extraction
import trajectory_extraction
import road_extraction
import planning_problem
from commonroad.common.file_reader import CommonRoadFileReader


# scenarios = ['105222', '120013', '128741', '148154', '156722']
scenarios = ['99817', '105222', '108812', '120305', '122168', '128697', '148154', '156722']

if __name__ == "__main__":
    for scenario in scenarios:
        input_file_path = 'output_' + scenario + '.json'
        trajectory_file_path = 'trajectories_' + scenario + '.json'
        empty_scenario = 'empty_scenario.xml'
        roads_file_path = 'roads_' + scenario + '.xml'
        output_file_path = 'scenario_' + scenario + '.xml'
        # road_extraction.extractRoads(input_file_path, empty_scenario, roads_file_path)
        # crash_extraction.extractVehicles(input_file_path, roads_file_path, output_file_path)
        trajectory_extraction.extractVehicles(trajectory_file_path, roads_file_path, output_file_path, 20.0)
        print("CRISCE scenario", scenario, "was successfully converted to CommonRoad scenario")
        # create one planning problem for each dynamic obstacle
        # s, p = CommonRoadFileReader(output_file_path).open()
        # for i in range(len(s.obstacles)):
        #     planning_problem.createPlanningProblem(output_file_path, s.obstacles[i].obstacle_id)
        # print("Successfully created", len(s.obstacles), "different planning problems for scenario " + scenario)
