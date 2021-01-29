cmake_minimum_required(VERSION 3.0.2)
project(gazebo_scene_randomiser_plugin)

add_compile_options(-std=c++11)

# this snippet for FCL is from https://github.com/ros-planning/moveit/blob/master/moveit_core/CMakeLists.txt
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBFCL_PC REQUIRED fcl)
set(LIBFCL_INCLUDE_DIRS ${LIBFCL_PC_INCLUDE_DIRS})
# find *absolute* paths to LIBFCL_LIBRARIES
set(LIBFCL_LIBRARIES)
foreach(_lib ${LIBFCL_PC_LIBRARIES})
  find_library(_lib_${_lib} ${_lib} HINTS ${LIBFCL_PC_LIBRARY_DIRS})
  list(APPEND LIBFCL_LIBRARIES ${_lib_${_lib}})
endforeach()

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  message_generation
  message_runtime
  roscpp
  std_msgs
)

find_package(gazebo REQUIRED)

add_service_files(
  FILES
  randomise.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES gazebo_scene_randomiser_plugin
  CATKIN_DEPENDS gazebo_ros roscpp std_msgs
#  DEPENDS system_lib
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/scene_randomiser_plugin.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)