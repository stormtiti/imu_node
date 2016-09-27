cmake_minimum_required(VERSION 2.8.3)
project(imu_node)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC -O3 -DDEBUG_OPTION")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -static-libgcc -static-libstdc++")

find_package(catkin REQUIRED COMPONENTS nav_msgs roscpp tf  rospy std_msgs  )

## Declare ROS messages and services




catkin_package()


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${keyboard_move_SOURCE_DIR}/libs/
  /lib/x86_64-linux-gnu/
)

# Build the executable
add_executable(imu_node  src/pub_imu.cpp )
# Add a build order dependency on nav_msgs
# This ensures that nav_msgs' msg headers are built before your executable
#if(nav_msgs_EXPORTED_TARGETS)
#  add_dependencies(keyboard_move ${nav_msgs_EXPORTED_TARGETS})
#endif()
# Link against the catkin libraries
target_link_libraries(imu_node ${catkin_LIBRARIES})


add_dependencies(imu_node beginner_tutorials_generate_messages_cpp)
# Install the executable
install(TARGETS imu_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
