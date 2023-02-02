
# STRETCH: Generating Avoidable Scenarios from Car Crash Simulations

This repository hosts the Master thesis of Franz Scheuer (University of Passau, Passau, Germany).

#### Supervisors: 
- Prof. Dr.-Ing. Gordon Fraser (University of Passau, Passau, Germany)
- Prof. Dr.-Ing. Christian Hammer (University of Passau, Passau, Germany)

#### Advisors: 
- Ph.D. Alessio Gambi (IMC University of Applied Sciences, Krems, Austria)
- Ph.D. Paolo Arcaini (National Institute of Informatics, Tokyo, Japan)

This project is able to analyze scenarios in the [CommonRoad](https://commonroad.in.tum.de/) scenario format. The results are used to generate realistic, avoidable and critical test cases and can be useful as feedback to testers and developers working on autonomous planning software.

The implementation uses the public available [CommonRoad](https://commonroad.in.tum.de/) framework. There are multiple tutorials and instructions on how to install and setup the different modules on their website. Essential modules and their used versions are:
- commonroad-drivability-checker  (2021.1)
- commonroad-io  (2021.3)
- commonroad-route-planner  (1.0.0)

I tested the approach with the "Reactive-Planner" by Prof. Dr.-Ing. Matthias Althoff (althoff@tum.de) and his team from the Technical University of Munich. Due to privacy reasons their code was fully removed from this repository. To use the project, please contact them for a full version of their planner. 
Many thanks to them for letting me use their software!

#### Project Structure:

**/results/**: evaluation results of the original planner
**/mutant/**: evaluation of the mutated (fault based) planner

- clustered by  scenarios
- each scenario folder is clustered by extension time
- each scenario folder contains additional information files/pictures

**/GIFs/**: several visualization examples

- scenario which is avoidable after a certain threshold (*CD_M_Static*)
	- standard scenario, standard configuration
	- at lower bound (max. reduction)
- scenario which is never avoidable (*CIRSCE_105222*)
	- standard scenario but WITH prediction
	- at lower bound with perfect knowledge (PF) and max. sampling (MS)
- scenario which is sometimes avoidable (*CRISCE_120305*)
	- same extension, alternative weights, NO CRASH
	- same extension, alternative weights, CRASH

**/reactive_planner/**:

- **commonroad_rp**: reactive planner (MISSING CODE)
- **avoidability.py**: compute avoidability (danger)
- **custom_nsga.py**: adjustet NSGA-II implementation
- **evaluation.py**: analyze the search and visualize the results
- **extension_primitives.py**: extension primitives for extending the lanelets
- **lanelet_extension.py**: extend and reduce the lanelets
- **main.py**: setup and run the project
- **reachability.py**: compute the area to reach for safety
- **run_combined_planner.py**: run the planner (MISSING CODE)
- **scenario_extension.py**: extend and reduce  complete scenario
- **search.py**: search setup
- **trajectory_extension.py**: extend and reduce  trajectories
- **vision_radius.py**: compute the current vision radius and predict obstacle trajectories

**/scenario_generation/**:

- **crash_extraction.py**: extract crash trajectories
- **crisce_plus.py**: extract and visualize scenarios
- **main.py**: run the scenario generation
- **planning_problem.py**: generate a planning problem
- **road_extraction.py**: extract lanelet network
- **trajectory_extraction.py**: extract planned trajectories
