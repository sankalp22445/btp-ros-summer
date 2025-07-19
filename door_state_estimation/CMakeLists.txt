cmake_minimum_required(VERSION 3.0.2)
project(door_state_estimation)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  turtlesim
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/belief_main_robot.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

