cmake_minimum_required(VERSION 3.0.2)
project(vlm_duckiebot_interface)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  duckietown_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vlm_duckiebot_interface
  CATKIN_DEPENDS rospy std_msgs sensor_msgs duckietown_msgs geometry_msgs
#  DEPENDS system_lib
)

# Python scripts in src/ are typically not compiled with CMake in the same way C++ is.
# Ensure they are executable and correctly installed.
install(PROGRAMS
  src/robot_vlm_client_node.py 
  # Add other scripts here if you create more
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# If you have launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# If you have config files
# install(DIRECTORY config/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
# )
