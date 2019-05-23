# test_scenario_runner

Python ROS node with a command-line interface that facilitates running simulated scenarios to test the 
path planner in the CCOM project11 simulation environment.

## Instructions
First start the path planner version of the project11 simulation environment:
```
roslaunch project11 sim_path_planner_local.launch
```
and let it finish starting before running the node.

The node works from a command line interface, so the best way to run it is from a terminal with <code>rosrun</code>:
```
rosrun test_scenario_runner test_scenario_runner_node.py
```

Once it's running it gives the self-explanatory prompt:
```
Enter a scenario filename to run a test, or 'done' to exit: 
```
Enter the path to a test file as specified below (and at the top of src/test_scenario_runner_node.py).

Alternatively, a suite of scenarios, described by a file ```sample.suite``` containing filenames of each included test, one per line, can be run in sequence:
```
cat sample.suite | rosrun test_scenario_runner test_scenario_runner_node.py 
```

### Test File Specification
Test file specification format is any number of lines containing one of the following:
```
point <x> <y>
obstacle <x> <y> <course over ground (degrees East of North)> <speed (m/s)>
map_file <path to grid-world-style file>
# Lines starting with "#" (or anything not specified above, actually) will be ignored
```

### Notes
Maps and obstacles are optional. Only the last map declared will be used. \
Maps are not yet supported, so file names in test files will be ignored for now. \
Files without any point declarations will have no effect. \
Points represent endpoints to track-lines. The planner will interpolate more points between them. \
Obstacles will maintain speed and course. Different sizes/shapes are not yet supported. \
Obstacle observation time is assumed to be the time the test begins. \
Obstacle course over ground is in degrees because it's easier to type cardinal directions as whole numbers. 
