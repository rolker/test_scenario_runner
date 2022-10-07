#!/usr/bin/env python
import math
import os
from datetime import datetime
import pickle
import rospy
import rosbag
import dynamic_reconfigure.client
from std_msgs.msg import Bool, String, Header
from marine_msgs.msg import Contact, NavEulerStamped, Helm
from geometry_msgs.msg import PointStamped, TwistStamped, Vector3, PoseStamped
from geographic_msgs.msg import GeoPoseStamped, GeoPointStamped
from geographic_visualization_msgs.msg import GeoVizItem, GeoVizPointList, GeoVizPolygon, GeoVizSimplePolygon
from asv_sim.srv import SetPose
import actionlib
import path_planner.msg
from alex_path_planner_common.msg import Stats, TaskLevelStats
from threading import Lock
import project11


class TestScenarioRunner:
    """Class to run test scenarios for the path planner.
    Scenarios are specified by file names read from stdin.
    Coordinates and speeds are in map coordinates, typically meters.
    The test file specification format is any number of lines containing one of the following:
    line <x1> <y1> <x2> <y2>
    start <x> <y> <heading (degrees East of North)> <speed>
    obstacle <x> <y> <course over ground (degrees East of North)> <speed> [<width> <length>]
    time_limit <seconds>
    map_file <path to grid-world-style file>
    parameter_file <path to parameter file>
    # Lines starting with "#" (or anything not specified above, actually) will be ignored

    Maps and obstacles are optional. Only the last map declared will be used.
    Files without any line declarations will have no effect.
    Lines represent survey lines.
    Obstacles will maintain speed and course. Width and length are optional - default values will be used if absent.
    Obstacle observation time is assumed to be the time the test begins.
    Obstacle course over ground is in degrees because it's easier to type cardinal directions as whole numbers.
    Tests will be terminated after time_limit seconds have elapsed. The default time limit is 600s.
    Like the map_file, only the last time limit declared will be used.
    Specify a *.map file included in the repo or create your own in the same format (first number is resolution).
    Blank lines at the end of map files are not allowed, as the planner takes the shortest line as the boundary.
    See default.scenario_config for parameter file example. Later parameter files will override parameters set in
    earlier ones, and missing parameters will use default values.

    Run the node using rosrun and enter test file names when prompted, or redirect in a suite of tests from a file,
    with one file name per line. Entering "." the line following a valid test file name will re-run the previous test.
    File names are all relative paths - it is recommended to run this node in the scenarios directory or your own
    directory containing test scenarios. Entering a file name ending in .scenario_config will update the default
    parameters, which persist between tests and are used for parameters not specified in the test. Entering a file
    name ending in .suite will treat each line in the file as though it were entered through stdin. Suite files can
    contain other suites, so beware of self-referential recursion.
    """

    def __init__(self):
        self.test_running = False
        self.test_name = ""
        self.start_time = self.end_time = 0
        self.default_time_limit = 600
        self.bag = None
        self.write_bags = True
        self.bag_lock = Lock()
        rospy.init_node('test_scenario_runner')
        self.reset_publisher = rospy.Publisher('sim_reset', Bool, queue_size=5, latch=True)
        self.piloting_mode_publisher = rospy.Publisher('project11/piloting_mode', String, queue_size=5, latch=True)
        # self.send_command_publisher = rospy.Publisher('/send_command', String, queue_size=5, latch=True)
        self.contact_publisher = rospy.Publisher('contact', Contact, queue_size=5)
        self.display_publisher = rospy.Publisher('project11/display', GeoVizItem, queue_size=10, latch=True)

        self.stats_subscriber = rospy.Subscriber('path_planner/stats', Stats, self.stats_callback)
        self.task_level_stats_subscriber = rospy.Subscriber('path_planner/task_level_stats', TaskLevelStats,
                                                            self.task_level_stats_callback)
        # bagging subs
        # uncomment lines to record more topics (and replace _type with the message type... I'm writing those in lazily)
        topics = [
                 # ["/asv_sim_node/parameter_descriptions", _type],
                 # ["/asv_sim_node/parameter_updates", _type],
                 # ["/base/camera/look_at", _type],
                 # ["/base/camera/look_at_mode", _type],
                 # ["/base/orientation", _type],
                 # ["/base/position", _type],
                 # ["/bearing", _type],
                 # ["/clock", Clock],
                 # ["/clock_factor", _type],
                 # ["/cmd_vel", _type],
                 # ["/cmg", _type],
                 ["contact", Contact],
                 # ["/controller_msgs", _type],
                 # ["/coverage", _type],
                 # ["/diagnostics", _type],
                 # ["/disturbance_estimate", _type],
                 # ["/flir_engine", _type],
                 ["heading", NavEulerStamped],
                 # ["/heartbeat", _type],
                 ["helm", Helm],
                 # ["/hover_action/cancel", _type],
                 # ["/hover_action/feedback", _type],
                 # ["/hover_action/goal", _type],
                 # ["/hover_action/parameter_descriptions", _type],
                 # ["/hover_action/parameter_updates", _type],
                 # ["/hover_action/result", _type],
                 # ["/hover_action/status", _type],
                 # ["/mbes_ping", _type],
                 # ["/mbr/9372/2510/margin_avg", _type],
                 # ["/mbr/9372/2510/margin_min", _type],
                 # ["/mission_manager/parameter_descriptions", _type],
                 # ["/mission_manager/parameter_updates", _type],
                 # ["/mission_manager/smach/container_init", _type],
                 # ["/mission_manager/smach/container_status", _type],
                 # ["/mission_manager/smach/container_structure", _type],
                 # ["/mission_plan", _type],
                 ["mpc/disturbance_estimate", Vector3],
                 # ["/mpc/parameter_descriptions", _type],
                 # ["/mpc/parameter_updates", _type],
                 # ["/mpc/reference_trajectory", _type],
                 # ["/obstacle_distance", _type],
                 # ["/origin", _type],
                 # ["/path_follower_action/cancel", _type],
                 # ["/path_follower_action/feedback", _type],
                 # ["/path_follower_action/goal", _type],
                 # ["/path_follower_action/result", _type],
                 # ["/path_follower_action/status", _type],
                 # ["/path_planner/parameter_descriptions", _type],
                 # ["/path_planner/parameter_updates", _type],
                 ["path_planner/stats", Stats],
                 ["path_planner/task_level_stats", TaskLevelStats],
                 # ["/path_planner_action/cancel", _type],
                 # ["/path_planner_action/feedback", _type],
                 # ["/path_planner_action/goal", _type],
                 # ["/path_planner_action/result", _type],
                 # ["/path_planner_action/status", _type],
                 ["position", GeoPointStamped],
                 ["position_map", PoseStamped],
                 # ["/posmv/orientation", _type],
                 # ["/posmv/position", _type],
                 # ["/project11/command", _type],
                 # ["/project11/desired_heading", _type],
                 # ["/project11/desired_speed", _type],
                 ["project11/display", GeoVizItem],
                 # ["/project11/mission_manager/command", _type],
                 # ["/project11/mission_manager/status", _type],
                 # ["/project11/piloting_mode", _type],
                 # ["/project11/response", _type],
                 # ["/radar", _type],
                 # ["/radar/HaloA/data", _type],
                 # ["/range", _type],
                 # ["/remote/0/cmd_vel", _type],
                 # ["/rosmon_1594406967160063576/state", _type],
                 # ["/rosout", _type],
                 # ["/rosout_agg", _type],
                 # ["/send_command", _type],
                 # ["/sim_reset", _type],
                 ["sog", TwistStamped],
                 # ["/speed_modulation", _type],
                 # ["/survey_area_action/cancel", _type],
                 # ["/survey_area_action/feedback", _type],
                 # ["/survey_area_action/goal", _type],
                 # ["/survey_area_action/result", _type],
                 # ["/survey_area_action/status", _type],
                 # ["/tf", _type],
                 # ["/tf_static", _type],
                 # ["/udp/command", _type],
                 # ["/udp/contact", _type],
                 # ["/udp/coverage", _type],
                 # ["/udp/diagnostics", _type],
                 # ["/udp/flir_engine", _type],
                 # ["/udp/heading", _type],
                 # ["/udp/heartbeat", _type],
                 # ["/udp/helm", _type],
                 # ["/udp/mbes_ping", _type],
                 # ["/udp/mbr/9372/2510/margin_avg", _type],
                 # ["/udp/mbr/9372/2510/margin_min", _type],
                 # ["/udp/mission_plan", _type],
                 # ["/udp/origin", _type],
                 # ["/udp/position", _type],
                 # ["/udp/posmv/orientation", _type],
                 # ["/udp/posmv/position", _type],
                 # ["/udp/project11/display", _type],
                 # ["/udp/project11/mission_manager/status", _type],
                 # ["/udp/project11/piloting_mode", _type],
                 # ["/udp/radar", _type],
                 # ["/udp/response", _type],
                 # ["/udp/sog", _type],
        ]
        self.bagging_subs = [
            rospy.Subscriber(topic[0], topic[1], self.get_bagging_callback(topic[0]), queue_size=10) for topic in topics
        ]

        #self.map_to_lat_long = rospy.ServiceProxy('map_to_wgs84', MapToLatLong)
        self.map_frame = 'ben/map'
        self.earth_transforms = project11.nav.EarthTransforms(map_frame=self.map_frame)
        self.set_pose = rospy.ServiceProxy('set_pose', SetPose)

        self.path_planner_client = actionlib.SimpleActionClient('path_planner_action',
                                                                path_planner.msg.path_plannerAction)

        self.path_planner_parameter_client = dynamic_reconfigure.client.Client("path_planner", timeout=10)
        self.mpc_parameter_client = dynamic_reconfigure.client.Client("mpc", timeout=10)
        self.asv_sim_parameter_client = dynamic_reconfigure.client.Client("asv_sim_node", timeout=10)

        # recording stats
        # so I don't get yelled at for defining fields outside of __init__
        self.stats = {}
        # so I don't have to have everything defined in more than one place
        self.reset_stats()

        self.default_planner_config = {
            "non_coverage_turning_radius": 8.0,
            "coverage_turning_radius": 100.0,
            "max_speed": 2.0,
            "slow_speed": 1.0,
            "line_width": 2.0,
            "branching_factor": 9,
            "time_horizon": 30.0,
            "time_minimum": 5.0,
            "collision_checking_increment": 0.05,
            "initial_samples": 100,
            "use_brown_paths": True,
            "dump_visualization": False,
            "visualization_file": "/tmp/planner_scenario_visualization",
            "heuristic": 0,
            "dynamic_obstacles": 0,
            "ignore_dynamic_obstacles": False,
            "planner": 0,
        }
        self.default_mpc_config = {
            "rudder_granularity": 0.0625,
            "throttle_granularity": 0.125,
            "distance_weight": 1.0,
            "heading_weight": 20.0,
            "speed_weight": 5.0,
            "achievable_threshold": 20.0,
            "current_estimation": True,
        }
        self.default_sim_config = {
            "current_speed": 0.5,
            "current_direction": 90.0,
            "jitter_thrust": 0.1,
            "jitter_rudder": 0.25,
            "jitter_drag": 0.1,
            "jitter_current_speed": 0.1,
            "jitter_current_direction": 0.25,
        }

        self.planner_config = self.default_planner_config
        self.mpc_config = self.default_mpc_config
        self.sim_config = self.default_sim_config

    def convert_point(self, x, y):
        ps = PointStamped()
        ps.point.x = x
        ps.point.y = y
        ps.header.frame_id = self.map_frame
        p = None
        while p is None:
            p = self.earth_transforms.pointToGeoPoint(ps)
            if p is None:
                print('did not convert point, trying again...')
                rospy.sleep(0.5)
        return p.position

    def convert_line(self, line):
        print("Converting line", line)
        assert len(line) == 4
        p1 = self.convert_point(line[0], line[1])
        p2 = self.convert_point(line[2], line[3])
        return [p1, p2]

    def done_callback(self, status, result):
        self.test_running = False

    def active_callback(self):
        pass

    def feedback_callback(self, msg):
        pass

    # love a good higher order function
    def get_bagging_callback(self, topic):
        def callback(msg):
            self.bag_msg(topic, msg)
        return callback

    def bag_msg(self, topic, msg):
        self.bag_lock.acquire()
        try:
            if self.bag and self.write_bags:
                self.bag.write(topic, msg)
        finally:
            self.bag_lock.release()

    def stats_callback(self, msg):
        self.stats["samples_counts"].append(msg.samples)
        self.stats["generated_counts"].append(msg.generated)
        self.stats["expanded_counts"].append(msg.expanded)
        self.stats["iterations_counts"].append(msg.iterations)
        self.stats["f_values"].append(msg.plan_f_value)
        self.stats["plan_collision_penalties"].append(msg.plan_collision_penalty)
        self.stats["plan_time_penalties"].append(msg.plan_time_penalty)
        self.stats["plan_h_values"].append(msg.plan_h_value)
        self.stats["plan_depths"].append(msg.plan_depth)
        self.stats["collision_penalties"].append(msg.collision_penalty)
        self.stats["cpu_times"].append(msg.cpu_time)
        self.stats["last_plan_achievables"].append(msg.last_plan_achievable)
        # self.bag_msg('/path_planner/stats', msg)

    def task_level_stats_callback(self, msg):
        self.stats["total_time_from_planner"] = msg.time
        self.stats["cumulative_collision_penalty"] = msg.collision_penalty
        self.stats["score"] = msg.score
        self.stats["uncovered_length"] = msg.uncovered_length
        # self.bag_msg('/path_planner/task_level_stats', msg)

    def reset_stats(self):
        self.stats = {
            "samples_counts": [],
            "generated_counts": [],
            "expanded_counts": [],
            "iterations_counts": [],
            "f_values": [],
            "plan_collision_penalties": [],
            "plan_time_penalties": [],
            "plan_h_values": [],
            "plan_depths": [],
            "collision_penalties": [],
            "cpu_times": [],
            "last_plan_achievables": [],
            "total_time_from_planner": -1,
            "cumulative_collision_penalty": 0,
            "score": 0,
            "uncovered_length": 0,
            "time_limit": self.default_time_limit,  # store time limit here so it gets recorded
        }

    def write_results(self, path):
        # write config
        with open(path + "/planner_config.pickle", "wb") as results_file:
            pickle.dump(self.planner_config, results_file)
        with open(path + "/mpc_config.pickle", "wb") as results_file:
            pickle.dump(self.mpc_config, results_file)
        with open(path + "/sim_config.pickle", "wb") as results_file:
            pickle.dump(self.sim_config, results_file)
        # write stats
        with open(path + "/stats.pickle", "wb") as results_file:
            pickle.dump(self.stats, results_file)

    def spin_until_done(self, obstacles):
        while self.test_running:
            self.publish_obstacles(obstacles)
            rospy.sleep(0.5)
            if self.end_time < rospy.get_time():
                self.test_running = False
                self.path_planner_client.cancel_goal()
                rospy.sleep(2)  # sleep for a couple seconds after cancel from timeout

    def publish_obstacles(self, obstacles):
        for i in range(len(obstacles)):
            obs = obstacles[i]

            # Advance obstacle along its course
            d = (rospy.get_time() - obs[6]) * obs[3]
            dx = d * math.sin(obs[2])  # swapped because it's east of north
            dy = d * math.cos(obs[2])
            obs[0] += dx
            obs[1] += dy
            obs[6] = rospy.get_time()

            # publish as a contact
            contact = Contact()
            contact.position = self.convert_point(obs[0], obs[1])
            contact.cog = contact.heading = obs[2]
            contact.sog = obs[3]
            contact.mmsi = i
            contact.dimension_to_stbd = obs[4] / 2
            contact.dimension_to_port = obs[4] / 2
            contact.dimension_to_bow = obs[5] / 2
            contact.dimension_to_stern = obs[5] / 2
            contact.header = Header()
            contact.header.stamp = rospy.Time.now()
            self.contact_publisher.publish(contact)
            # self.bag_msg('/contact', contact)

    def load_parameters(self, parameter_file_names):
        self.planner_config = self.default_planner_config
        self.mpc_config = self.default_mpc_config
        self.sim_config = self.default_sim_config
        for parameter_file_name in parameter_file_names:
            try:
                with open(parameter_file_name, "r") as parameter_file:
                    for line in parameter_file:
                        name, value = line.split(' ')
                        for parameters in [self.planner_config, self.mpc_config, self.sim_config]:
                            if name in parameters:
                                parameters[name] = type(parameters[name])(value)
            except IOError as err:
                print ("Couldn't find default configuration file: " + parameter_file_name)
        self.path_planner_parameter_client.update_configuration(self.planner_config)
        self.mpc_parameter_client.update_configuration(self.mpc_config)
        self.asv_sim_parameter_client.update_configuration(self.sim_config)

    def update_default_parameters(self, parameter_file_name):
        try:
            with open(parameter_file_name, "r") as parameter_file:
                for line in parameter_file:
                    name, value = line.split(' ')
                    for parameters in [self.default_planner_config,
                                       self.default_mpc_config,
                                       self.default_sim_config]:
                        if name in parameters:
                            parameters[name] = type(parameters[name])(value)
                print("Default parameters updated.")
        except IOError as err:
            print ("Couldn't find default configuration file: " + parameter_file_name)

    def update_single_default_parameter(self, line):
        try:
            name, value = line.split(' ')
            if name == "time_limit":
                self.default_time_limit = int(value)
                print ("Default time limit updated")
                return True
            for parameters in [self.default_planner_config,
                               self.default_mpc_config,
                               self.default_sim_config]:
                if name in parameters:
                    parameters[name] = type(parameters[name])(value)
                    print ("Parameter " + name + " updated.")
                    return True
            return False
        except ValueError:
            return False

    def run_test(self, filename):
        lines = []
        index = 0
        obstacles = []
        map_file = ""
        start = []
        self.stats["time_limit"] = self.default_time_limit
        parameter_file_names = []
        try:
            with open(filename, "r") as testfile:
                for line in testfile:
                    line = line.strip()
                    if line.startswith("line"):
                        lines.append([float(f) for f in line.split(" ")[1:]])
                        print("Read line ", lines[-1])
                        assert len(lines[-1]) == 4  # should be startX startY endX endY
                    elif line.startswith("start"):
                        start = [float(f) for f in line.split(" ")[1:]]
                        start[2] = math.radians(start[2])
                        # assert len(start) == 4  # x y heading speed # assume speed is zero?
                    elif line.startswith("obstacle"):
                        obs = [float(f) for f in line.split(" ")[1:]]
                        if len(obs) == 4:
                            obs.append(5)
                            obs.append(20)
                        obstacles.append(obs)
                    # ignore test-defined time limit (deprecated)
                    # elif line.startswith("time_limit"):
                    #     self.stats["time_limit"] = float(line[10:])
                    elif line.startswith("map_file"):
                        map_file = line[8:].strip()
                    elif line.startswith("parameter_file"):
                        parameter_file_names.append(line[15:])
        except IOError as err:
            print ("Couldn't find file: " + filename)
            return
        try:
            # Convert to lat long
            lines = [self.convert_line(line) for line in lines]
        except rospy.ServiceException as exc:
            print("Map to LatLong service did not process request: " + str(exc))

        # # wait until clock is initialized
        # while rospy.get_time() == 0:
        #     pass
        if rospy.get_time() == 0:
            print ("Simulation does not appear to be running yet. Exiting.")
            return

        # load parameters and update dynamic reconfigure
        self.load_parameters(parameter_file_names)

        # load map file, if any
        if map_file:
            current_path = os.getcwd()
            self.path_planner_parameter_client.update_configuration({"planner_geotiff_map": current_path +
                                                                                            "/" + map_file})
        else:
            self.path_planner_parameter_client.update_configuration({"planner_geotiff_map": ""})

        self.test_name = filename

        # create results directory
        now = datetime.now()
        results_dir_path = "../results/" + now.strftime("%Y_%m_%d/%H:%M:%S_" + self.test_name)
        if not os.path.exists(results_dir_path):
            os.makedirs(results_dir_path)

        # initialize bag
        self.bag_lock.acquire()
        self.bag = rosbag.Bag(results_dir_path + "/logs.bag", 'w')
        self.bag_lock.release()

        self.piloting_mode_publisher.publish("autonomous")

        # Set up set_pose request
        if start:
            gps = GeoPointStamped()
            gps.position = self.convert_point(start[0], start[1])
            h = NavEulerStamped()
            h.orientation.heading = start[2]
            self.set_pose(gps, h)
        else:
            print ("Warning: no start state read; using default start state")
            self.reset_publisher.publish(True)

        rospy.sleep(0.2)  # Let simulator reset

        # finish setting up obstacles
        for o in obstacles:
            o.append(rospy.get_time())
            o[2] = math.radians(o[2])

        self.start_time = rospy.get_time()
        self.end_time = self.start_time + self.stats["time_limit"]

        # set up both path_planner goal and display for lines
        goal = path_planner.msg.path_plannerGoal()
        goal.path.header.stamp = rospy.Time.now()
        display_item = GeoVizItem()
        display_item.id = "current_path"

        for line in lines:
            # now sending all lines at once, so points are to be read in pairs
            display_points = GeoVizPointList()
            display_points.color.r = 1.0
            display_points.color.a = 1.0
            display_points.size = 5.0
            for p in line:
                pose = GeoPoseStamped()
                pose.pose.position = p
                goal.path.poses.append(pose)
                display_points.points.append(p)
            # TODO! -- goal.speed?
            display_item.lines.append(display_points)

        self.display_publisher.publish(display_item)
        self.test_running = True

        self.path_planner_client.wait_for_server()
        self.path_planner_client.send_goal(goal, self.done_callback, self.active_callback, self.feedback_callback)
        # Spin and publish obstacle updates every 0.5s
        self.spin_until_done(obstacles)

        print ("Test %s complete in %s seconds." % (self.test_name, (rospy.get_time() - self.start_time)))

        # remove line from display
        if display_item:
            display_item.lines = []
            self.display_publisher.publish(display_item)
        self.piloting_mode_publisher.publish("standby")

        # reporting

        self.write_results(results_dir_path)

        self.reset_stats()

        self.bag_lock.acquire()
        self.bag.close()
        self.bag = None
        self.bag_lock.release()


def run(input_text):
    if input_text == "done" or input_text == "exit":
        return False
    elif input_text == "." and runner.test_name != "":
        runner.run_test(runner.test_name)
    elif input_text.endswith(".scenario_config"):
        runner.update_default_parameters(input_text)
    elif runner.update_single_default_parameter(input_text):
        pass  # don't do anything because the parameter was updated
    elif input_text.endswith(".suite"):
        try:
            with open(input_text, "r") as suite_file:
                lines = [line for line in suite_file]
            for line in lines:
                # recur on a suite file
                run(line.strip())
        except IOError as err:
            print ("Couldn't find suite file: " + input_text)
    else:
        runner.run_test(input_text)
    return True


if __name__ == '__main__':
    runner = TestScenarioRunner()
    rospy.sleep(0.05)
    try:
        while True:
            try:
                filename = input("Enter a scenario filename to run a test, or 'done' to exit:\n")
            except EOFError:
                break
            rospy.sleep(0.1)
            if not run(filename):
                break
    except rospy.exceptions.ROSInterruptException:
        runner.path_planner_client.cancel_goal()
