cmake_minimum_required(VERSION 2.8.3)
project(simulated_omni_drive_skill_server)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  rospy
  roscpp
  simulated_omni_drive_skill_msgs
)

catkin_python_setup()

catkin_package(
 CATKIN_DEPENDS rospy actionlib simulated_omni_drive_skill_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(
  PROGRAMS scripts/simulated_omni_drive_skill.py 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

