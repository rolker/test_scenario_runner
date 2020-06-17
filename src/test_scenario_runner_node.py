#!/usr/bin/env python
import math

import rospy
from std_msgs.msg import Bool, String, Header
# from std_msgs.msg import String
from marine_msgs.msg import Contact, NavEulerStamped
from geometry_msgs.msg import PointStamped
from geographic_msgs.msg import GeoPoseStamped, GeoPointStamped
from project11_transformations.srv import MapToLatLong
from geographic_visualization_msgs.msg import GeoVizItem, GeoVizPointList
from asv_sim.srv import SetPose
import actionlib
import path_planner.msg


class TestScenarioRunner:
    """Class to run test scenarios for the path planner.
    Scenarios are specified by file names read from stdin.
    Coordinates and speeds are in map coordinates, typically meters.
    The test file specification format is any number of lines containing one of the following:
    line <x1> <y1> <x2> <y2>
    start <x> <y> <heading (degrees East of North)> <speed>
    obstacle <x> <y> <course over ground (degrees East of North)> <speed>
    time_limit <seconds>
    map_file <path to grid-world-style file>
    # Lines starting with "#" (or anything not specified above, actually) will be ignored

    Maps and obstacles are optional. Only the last map declared will be used.
    Files without any line declarations will have no effect.
    Lines represent survey lines.
    Obstacles will maintain speed and course. Different sizes/shapes are not yet supported.
    Obstacle observation time is assumed to be the time the test begins.
    Obstacle course over ground is in degrees because it's easier to type cardinal directions as whole numbers.
    Tests will be terminated after time_limit seconds have elapsed. The default time limit is 3600s.
    Like the map_file, only the last time limit declared will be used.
    Maps are not yet supported, so specifying map files will do nothing.

    Run the node using rosrun and enter test file names when prompted, or redirect in a suite of tests from a file,
    with one file name per line. Entering "." the line following a valid test file name will re-run the previous test.
    """
    def __init__(self):
        self.test_running = False
        self.test_name = ""
        self.start_time = self.end_time = 0
        rospy.init_node('test_scenario_runner')
        self.reset_publisher = rospy.Publisher('/sim_reset', Bool, queue_size=5, latch=True)
        self.piloting_mode_publisher = rospy.Publisher('/project11/piloting_mode', String, queue_size=5, latch=True)
        # self.send_command_publisher = rospy.Publisher('/send_command', String, queue_size=5, latch=True)
        self.contact_publisher = rospy.Publisher('/contact', Contact, queue_size=5)
        self.display_publisher = rospy.Publisher('/project11/display', GeoVizItem, queue_size=10, latch=True)

        self.map_to_lat_long = rospy.ServiceProxy('map_to_wgs84', MapToLatLong)  # TODO! -- is this right?
        self.set_pose = rospy.ServiceProxy('set_pose', SetPose)

        self.path_planner_client = actionlib.SimpleActionClient('path_planner_action', path_planner.msg.path_plannerAction)

    def convert_point(self, x, y):
        ps = PointStamped()
        ps.point.x = x
        ps.point.y = y
        return self.map_to_lat_long(ps).wgs84.position

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

    def spin_until_done(self, obstacles):
        while self.test_running:
            self.publish_obstacles(obstacles)
            rospy.sleep(0.5)
            if self.end_time < rospy.get_time():
                self.test_running = False
                self.path_planner_client.cancel_goal()

    def publish_obstacles(self, obstacles):
        for i in range(len(obstacles)):
            obs = obstacles[i]
            contact = Contact()
            contact.position = self.convert_point(obs[0], obs[1])
            contact.cog = contact.heading = obs[2]
            contact.sog = obs[3]
            contact.mmsi = i
            contact.header = Header()
            contact.header.stamp = rospy.Time.now()
            self.contact_publisher.publish(contact)

            # Advance obstacle along its course
            d = (rospy.get_time() - obs[4]) * obs[3]
            dx = d * math.sin(obs[2])  # swapped because it's east of north
            dy = d * math.cos(obs[2])
            obs[0] += dx
            obs[1] += dy
            obs[4] = rospy.get_time()

    def run_test(self, filename):
        lines = []
        index = 0
        obstacles = []
        map_file = ""
        start = []
        period = None
        time_limit = 600
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
                        # assert len(start) == 4  # x y heading speed # assume speed is zero?
                    elif line.startswith("obstacle"):
                        obstacles.append([float(f) for f in line.split(" ")[1:]])
                        if period is not None:
                            obstacles[-1].append(period)
                    elif line.startswith("time_limit"):
                        time_limit = float(line[10:])
                    elif line.startswith("map_file"):
                        map_file = line[8:].strip()
                    elif line.startswith("period"):
                        if line[6:] == "-1":
                            period = None
                        else:
                            period = float(line[6:])
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
        self.test_name = filename

        self.start_time = rospy.get_time()
        self.end_time = self.start_time + time_limit

        display_item = None

        for line in lines:
            if self.end_time < rospy.get_time():
                return

            for o in obstacles:
                o.append(rospy.get_time())
                o[2] = math.radians(o[2])

            # Ask the planner to cover the points
            goal = path_planner.msg.path_plannerGoal()
            goal.path.header.stamp = rospy.Time.now()
            display_item = GeoVizItem()
            display_item.id = "current_path"
            display_points = GeoVizPointList()
            display_points.color.r = 1.0
            display_points.color.a = 1.0
            display_points.size = 5.0
            for p in line:
                pose = GeoPoseStamped()
                pose.pose.position = p
                goal.path.poses.append(pose)
                display_points.points.append(p)
            # TODO! -- goal.speed?  # ?? this might not be relevant anymore
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


if __name__ == '__main__':
    runner = TestScenarioRunner()
    rospy.sleep(0.05)
    try:
        while True:
            try:
                filename = raw_input("Enter a scenario filename to run a test, or 'done' to exit:\n")
            except EOFError:
                break
            rospy.sleep(0.1)
            if filename == "done" or filename == "exit":
                break
            elif filename == "." and runner.test_name != "":
                runner.run_test(runner.test_name)
            else:
                runner.run_test(filename)
    except rospy.exceptions.ROSInterruptException:
        runner.path_planner_client.cancel_goal()
