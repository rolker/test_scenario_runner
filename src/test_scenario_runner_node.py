#!/usr/bin/env python
import math

import rospy
from std_msgs.msg import Bool, String, Header
# from std_msgs.msg import String
from marine_msgs.msg import Contact
from geometry_msgs.msg import PointStamped
from geographic_msgs.msg import GeoPoseStamped
from project11_transformations.srv import MapToLatLong
import actionlib
import path_planner.msg

# Test file specification format is any number of lines containing one of the following:
# point <x> <y>
# obstacle <x> <y> <course over ground (degrees East of North)> <speed (m/s)>
# map_file <path to grid-world-style file>
# # Lines starting with "#" (or anything not specified above, actually) will be ignored

# Maps and obstacles are optional. Only the last map declared will be used.
# Files without any point declarations will have no effect.
# Points represent endpoints to track-lines. The planner will interpolate more points between them.
# Obstacles will maintain speed and course. Different sizes/shapes are not yet supported.
# Obstacle observation time is assumed to be the time the test begins.
# Obstacle course over ground is in degrees because it's easier to type cardinal directions as whole numbers.


class TestScenarioRunner:
    def __init__(self):
        self.test_running = False
        self.test_name = ""
        self.start_time = 0
        rospy.init_node('test_scenario_runner')
        self.reset_publisher = rospy.Publisher('/sim_reset', Bool, queue_size=5, latch=True)
        self.send_command_publisher = rospy.Publisher('/send_command', String, queue_size=5, latch=True)
        self.contact_publisher = rospy.Publisher('/contact', Contact, queue_size=5)

        self.map_to_lat_long = rospy.ServiceProxy('map_to_wgs84', MapToLatLong)  # TODO! -- is this right?

        self.path_planner_client = actionlib.SimpleActionClient('path_planner_action', path_planner.msg.path_plannerAction)

    def convert_point(self, x, y):
        ps = PointStamped()
        ps.point.x = x
        ps.point.y = y
        return self.map_to_lat_long(ps).wgs84.position

    def done_callback(self, status, result):
        self.test_running = False
        print ("Test %s complete in %s seconds." % (self.test_name, (rospy.get_time() - self.start_time)))

    def active_callback(self):
        pass

    def feedback_callback(self, msg):
        pass

    def spin_until_done(self, obstacles):
        while self.test_running:
            self.publish_obstacles(obstacles)
            rospy.sleep(0.5)

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
        points = []
        obstacles = []
        map_file = ""
        try:
            with open(filename, "r") as testfile:
                for line in testfile:
                    line = line.strip()
                    if line.startswith("point"):
                        points.append([float(f) for f in line.split(" ")[1:]])
                    elif line.startswith("obstacle"):
                        obstacles.append([float(f) for f in line.split(" ")[1:]])
                    elif line.startswith("map_file"):
                        map_file = line[8:].strip()
        except IOError as err:
            print ("Couldn't find file: " + filename)
            return
        try:
            points = [self.convert_point(p[0], p[1]) for p in points]
        except rospy.ServiceException as exc:
            print("Map to LatLong service did not process request: " + str(exc))

        if not points:  # no points specified
            return

        # wait until clock is initialized
        while rospy.get_time() == 0:
            pass

        for o in obstacles:
            o.append(rospy.get_time())
            o[2] = math.radians(o[2])
        self.send_command_publisher.publish("helm_mode autonomous")
        self.reset_publisher.publish(True)
        self.test_name = filename

        # Ask the planner to cover the points
        goal = path_planner.msg.path_plannerGoal()
        goal.path.header.stamp = rospy.Time.now()
        for p in points:
            pose = GeoPoseStamped()
            pose.pose.position = p
            goal.path.poses.append(pose)
        # TODO! -- goal.speed?
        self.test_running = True
        self.start_time = rospy.get_time()
        self.path_planner_client.wait_for_server()
        self.path_planner_client.send_goal(goal, self.done_callback, self.active_callback, self.feedback_callback)
        # Spin and publish obstacle updates every 0.1s
        self.spin_until_done(obstacles)


if __name__ == '__main__':
    runner = TestScenarioRunner()
    try:
        while True:
            try:
                filename = raw_input("Enter a scenario filename to run a test, or 'done' to exit:\n")
            except EOFError:
                break
            if filename == "done" or filename == "exit":
                break
            else:
                runner.run_test(filename)
    except rospy.exceptions.ROSInterruptException:
        pass
