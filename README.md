# STRETCH: Generating Avoidable Scenarios from Car Crash Simulations

This repository hosts the Master thesis of Franz Scheuer (University of Passau, Passau, Germany).

#### Supervisors: 
- Prof. Dr.-Ing. Gordon Fraser (University of Passau, Passau, Germany)
- Prof. Dr.-Ing. Christian Hammer (University of Passau, Passau, Germany)

#### Advisors: 
- Ph.D. Alessio Gambi (IMC University of Applied Sciences, Krems, Austria)
- Ph.D. Paolo Arcaini (National Institute of Informatics, Tokyo, Japan)

This project is able to analyze scenarios in the [CommonRoad](https://commonroad.in.tum.de/) scenario format. The results are used to generate avoidable AND critical test cases and can be useful to test planning software.

The implementation uses the public available [CommonRoad](https://commonroad.in.tum.de/) framework. There are multiple tutorials and instructions on how to install and setup the different modules on their website.

I tested the approach with the "Reactive-Planner" by Matthias Althoff (althoff@tum.de) and his team from the Technical University of Munich. Due to privacy reasons their code was fully removed from this repository. To use the project, please contact them for a full version of their planner. 
Many thanks to them for letting me use their software!

#### Project Structure:

**/results/**: evaluation results of the original planner

**/mutant/**: evaluation of the mutated (fault based) planner

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
