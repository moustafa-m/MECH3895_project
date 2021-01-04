cmake_minimum_required(VERSION 3.0)
project(planner)

set(CMAKE_BUILD_TYPE "DEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DDEBUG")

add_compile_options(-std=c++14)

find_package(ompl REQUIRED)
message(STATUS "Found OMPL ver: ${OMPL_VERSION}")

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBFCL_PC REQUIRED fcl)
set(LIBFCL_INCLUDE_DIRS ${LIBFCL_PC_INCLUDE_DIRS})
# find *absolute* paths to LIBFCL_LIBRARIES (snippet from MoveIt! repo moveit_core/CMakeLists.txt)
set(LIBFCL_LIBRARIES)
foreach(_lib ${LIBFCL_PC_LIBRARIES})
  find_library(_lib_${_lib} ${_lib} HINTS ${LIBFCL_PC_LIBRARY_DIRS})
  list(APPEND LIBFCL_LIBRARIES ${_lib_${_lib}})
endforeach()


find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  trajectory_msgs
  control_msgs
  actionlib_msgs
  actionlib
  gazebo_msgs
  sensor_msgs
  trac_ik_lib
  gazebo_geometries_plugin
)

find_package(gazebo REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


add_service_files(
  FILES
  start_plan.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES planner
 CATKIN_DEPENDS roscpp trajectory_msgs control_msgs actionlib_msgs trac_ik_lib gazebo_geometries_plugin
#  DEPENDS system_lib
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${fcl_LIBRARY_DIRS}
  ${OMPL_LIBRARY_DIRS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${fcl_INCLUDE_DIRS}
)

add_executable(planner_node src/node.cpp src/controller.cpp src/planner.cpp src/manipulator.cpp)
add_dependencies(planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(planner_node
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  fcl
  Eigen3::Eigen
)