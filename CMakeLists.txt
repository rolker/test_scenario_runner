cmake_minimum_required(VERSION 2.8.3)
project(test_scenario_runner)

find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        geographic_msgs
        project11_msgs
        rospy
        )

catkin_python_setup()

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES asv_sim
        #  CATKIN_DEPENDS asv_msgs asv_srvs rospy
        #  DEPENDS system_lib
)

include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS src/test_scenario_runner_node.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
