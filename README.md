# test_scenario_runner

Python ROS node with a command-line interface that facilitates running simulated scenarios to test the 
path planner in the CCOM project11 simulation environment.

### How it works
The test runner reads a test file, which describes everything about the test scenario, including the static map, survey lines, dynamic obstacles, ASV initial pose, and system configurations.
The test runner then calls the path planner with the survey line, and publishes updates about the dynamic obstacles periodically (loosely simulating AIS).
When the planner finishes the scenario or reaches the time limit, the test runner records some information about the test run.

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
File names are all relative paths; it is thus recommended to run this node in the <code>scenarios</code> directory or your own
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
Proper parsing and error reporting is a luxury not yet supported. Double check your test specification file(s) if your test isn't looking right.

##### Notes
Maps and obstacles are optional. Only the last map declared will be used. \
Files without any line declarations will have no effect.\
Lines represent survey lines.\
Obstacles will maintain speed and course. Width and length are optional - default values will be used if absent.\
Obstacle observation time is assumed to be the time the test begins.\
Obstacle course over ground is in degrees because it's easier to type cardinal directions as whole numbers.\
Tests will be terminated after `time_limit` seconds have elapsed. The default time limit is 3600s.\
Like the `map_file`, only the last time limit declared will be used.\
Specify a `.map` file included in the repo or create your own in the same format (first number is resolution).\
Blank lines at the end of map files are not allowed, as the planner takes the shortest line as the boundary.\
See `default.scenario_config` for parameter file example. Later parameter files will override parameters set in
earlier ones, and missing parameters will use default values.\

### Logging results
The test runner logs some information about each test it runs. 
This information is stored in sub-directories of the relative (to where the node is run) path `../results/`, organized by the date and time of the test start. 
If you want these stored somewhere else, please edit the source of the node.

Logged information includes a ROS bag of several topics, including `/helm`, for controller output, `/contact`, for dynamic obstacles, as well as all the high level ASV pose-related topics. Add or remove topics by editing the source of the node (there's a big list of topics; uncomment the ones you want).

Other information that the planner publishes is also recorded separately, for ease of plotting.
Some examples of plotting scripts, which read these data, are available in this repository, and should be a reasonable starting point for your own plots. Look at `results_loader.py` for simply reading the data, if you're already familiar with plotting in python, or want to use a different language/framework.

