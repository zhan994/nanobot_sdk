cmake_minimum_required(VERSION 3.0.2)
project(l150_sdk)

add_compile_options(-std=c++14 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf
)

include_directories(${catkin_INCLUDE_DIRS})

# Target names are package-prefixed so this SDK can coexist with the legacy
# packages during migration. OUTPUT_NAME preserves the public rosrun names.
add_executable(l150_sdk_serial_node src/l150_serial_node.cpp)
set_target_properties(l150_sdk_serial_node PROPERTIES OUTPUT_NAME l150_serial_node)
target_link_libraries(l150_sdk_serial_node ${catkin_LIBRARIES})

add_executable(l150_sdk_odometry_node src/l150_odometry_node.cpp)
set_target_properties(l150_sdk_odometry_node PROPERTIES OUTPUT_NAME l150_odometry_node)
target_link_libraries(l150_sdk_odometry_node ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
  node/evdev_joy_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
  l150_sdk_serial_node
  l150_sdk_odometry_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config doc img launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
