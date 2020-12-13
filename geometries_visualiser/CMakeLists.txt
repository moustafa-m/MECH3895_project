cmake_minimum_required(VERSION 3.0.2)
project(geometries_visualiser)

set(CMAKE_BUILD_TYPE "DEBUG")
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gazebo_geometries_plugin
  gazebo_msgs
  roscpp
)


catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES geometries_visualiser
 CATKIN_DEPENDS gazebo_geometries_plugin gazebo_msgs roscpp
#  DEPENDS system_lib
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_executable(visualiser src/geometries_visualiser.cpp src/visualiser_node.cpp)
add_dependencies(visualiser ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(visualiser
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
)