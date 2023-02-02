# STRETCH: Generating Challenging Scenarios for Testing Collision Avoidance Systems

This repository hosts the code, results, and visualization of our appraoch to generate critical scenarios from real crash data.
STRETCH analyzes scenarios in the [CommonRoad](https://commonroad.in.tum.de/) scenario format and generates realistic, avoidable and critical test cases that can be useful to testers and developers working on motion planning software.

### Authors

- Franz Scheuer (University of Passau, Passau, Germany)
- Ph.D. Alessio Gambi (IMC University of Applied Sciences, Krems, Austria)
- Ph.D. Paolo Arcaini (National Institute of Informatics, Tokyo, Japan)

## Abstract

Collision avoidance systems are fundamental for autonomous driving and need to be tested thoroughly to check whether they can safely handle critical scenarios. Testing collision avoidance systems is generally done by means of scenario-based testing using simulators and comes with the main open challenge of generating driving situations that are realistic, critical, but avoidable. In other words, driving scenarios must stress the collision avoidance functionalities while being representative.

Existing crash databases and accident reports describe the pre-crash, the impact, and the aftermath of observed car accidents, and enable to (re)create realistic collisions in simulations; however, because those data sources focus on the impact, their data do not generally lead avoidable collision scenarios.

To address this issue, we propose STRETCH, an approach that generates realistic, critical, but avoidable, collision scenarios by extending focused collision descriptions using a multi-objective optimization algorithm. Thanks to STRETCH developers and testers can automatically generate challenging test cases based on realistic crash scenarios.

## Dependencies

The implementation uses the public available [CommonRoad](https://commonroad.in.tum.de/) framework. There are multiple tutorials and instructions on how to install and setup the different modules on their website. Essential modules and their used versions are:
- commonroad-drivability-checker  (2021.1)
- commonroad-io  (2021.3)
- commonroad-route-planner  (1.0.0)

We validated our approach using the "Reactive-Planner" kindly provided by Prof. Dr.-Ing. Matthias Althoff's group at the Technical University of Munich. Due to privacy reasons the code of the planner cannot be released in this repository. Please contact Prof. Althoff to obtain the planner. 

## Repository Structure

**/results/**: evaluation results of the original planner\
**/mutant/**: evaluation of the mutated (fault based) planner

- clustered by  scenarios
- each scenario folder is clustered by extension time
- each scenario folder contains additional information files and pictures

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
