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
rosrun test_scenario_runner test_scenario_runner_node.py < sample.suite
```
Entering "." the line following a valid test file name will re-run the previous test.
File names are all relative paths - it is recommended to run this node in the scenarios directory or your own
directory containing test scenarios. Entering a file name ending in .scenario_config will update the default
parameters, which persist between tests and are used for parameters not specified in the test. Entering a file
name ending in .suite will treat each line in the file as though it were entered through stdin. Suite files can
contain other suites, so beware of self-referential recursion.

### Test File Specification
Test file specification format is any number of lines containing one of the following:
```
line <x1> <y1> <x2> <y2>
start <x> <y> <heading (degrees East of North)> <speed>
obstacle <x> <y> <course over ground (degrees East of North)> <speed> [<width> <length>]
time_limit <seconds>
map_file <path to grid-world-style file>
parameter_file <path to parameter file>
# Lines starting with "#" (or anything not specified above, actually) will be ignored
```

### Notes
Maps and obstacles are optional. Only the last map declared will be used. \
Files without any line declarations will have no effect.\
Lines represent survey lines.\
Obstacles will maintain speed and course. Width and length are optional - default values will be used if absent.\
Obstacle observation time is assumed to be the time the test begins.\
Obstacle course over ground is in degrees because it's easier to type cardinal directions as whole numbers.\
Tests will be terminated after time_limit seconds have elapsed. The default time limit is 3600s.\
Like the map_file, only the last time limit declared will be used.\
Specify a .map file included in the repo or create your own in the same format (first number is resolution).\
Blank lines at the end of map files are not allowed, as the planner takes the shortest line as the boundary.\
See default.scenario_config for parameter file example. Later parameter files will override parameters set in
earlier ones, and missing parameters will use default values.\
