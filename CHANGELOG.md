# Changelog

Noteable/major changes will be listed here

## 24th Jan 2021

### Added

- Integrated ROS params into Planner and Manipulator classes
    - initParams() method for Manipulator class added
    - If the display_path param is set to true but save_path is set to false, the path will still be saved but will be removed after it is plotted.
- Overloaded == and != operators for CollisionGeometry struct

### Modified

- Changed default planner params.

### Removed

- setLimits() method in Manipulator class. Was completely unnecessary, don't know why I even implemented it?
- waitForGazebo() method in node.cpp. It was redundant, so functionality is implemented in main() instead.
- Error and Debug prints from solveIK() and solveFK() methods in Manipulator class. Errors will be handled with assert() and by user calling the method.

## 14th Jan 2021

### Added

- getJointStates() method in Manipulator class
- setTargetName(), setTargetCollision(), and setNonStaticCollision() methods for StateChecker class
- setTargetGeometry(), isObjectBlocked(), and planInClutter() methods to Planner class
- StateChecker can disable/enable checks for target and non-static object collisions using the previously mentioned methods
- check for Gazebo node if starting planner node without Gazebo being launched first
- prefixes to print messages to identify the class that printed it (e.g. [PLANNER]: for planner related code)
- limited support for clutter clearing using push actions
- Controller class checks for current state of Kinova (init and home position check, and gripper open/close checks)
- templated functions approxEqual() and clamp() in util.h

### Modified

- Planner node uses ros::MultiThreadedSpinner with 3 threads instead of ros::spinOnce() with pre-defined rate
- arm no longer moves to init when node starts. When a planning request is sent, the arm moves to the init position
- StateChecker class integrated in Planner class to allow its use to check states without having to start planning a path
- IK solver timeout reduced to 0.05s
- Path markers are now coloured differently
- Target marker is now shown with the same geometry as the object (i.e. it appears exactly like the object's collision geometry)

### Removed

- unused headers in planner.h
- Kinova will not return to home by default after a successful planning attempt

## 11th Jan 2021

### Added

- Launch file for the planner node

### Modified

- Changed parameters in [planner/params/params.yaml](planner/params/params.yaml). The parameters are now for the planner and kinematics solver but are not yet used in the node.

## 10th Jan 2021

### Modified

- Moved Gazebo params file and rviz files to kinova_gazebo package. The planner package still retains the gazebo.launch file however it now just calls the robot_launch.h file in the kinova_gazebo package
- Gazebo params file is now called gazebo_params.yaml

### Removed

- optimal_test.world and kinova.rviz files. (they were only used to test out functionality)

## 28th Dec 2020

### Added

- Services for opening and closing the gripper

### Fixed

- Bug that occured when creating a new problem without terminating the node. The planner used the path for the previous solution rather than the newly obtained one. Solution paths and planner are now cleared prior to any new planning problem.

## 26th Dec 2020

### Added

- start_plan srv that takes in a target name implemented in Controller class, this is the main way to interface with the class and start planning for a given target
- Manipulator class is now passed to Planner and State Validity via pointer
- ob::PlannerPtr private member in Planner class
- Rviz marker publish methods to Planner class

### Modified

- Rviz marker publishing is now handled by the Planner class
- Planner plan() method returns bool and a trajectory via pass-by-reference
- Controller openGripper(), closeGripper(), and sendAction() are now public
- Collision box y-axis dimension for the Kinova finger reduced, it does not encompass the entire finger now. This is to avoid issues in edge cases where there is slight overlap with the box but the finger (a consequence of using a box shape as the collision geometry)
- State validity uses transforms between end effector and fingers and links to obtain poses of collision geometries
- IK solver timeout reduced to 0.1 seconds

### Removed

- Planner setGoal() no longer needs target name as input
- Controller run() method
- Controller setTargetName() method
- Controller getJointGoal() method
- Planner setManipulatorName()

## 20th Dec 2020

### Modified

- OMPL path waypoints are now represented by arrows which point in the same direction of the end effector

## 19th Dec 2020

### Modified

- Manipulator FK solver now returns all joint poses and can take in a set of joint positions (if no input given, it uses current joint positions)
- Manipulator IK method no longer exits program if it fails
- Manipulator IK and FK methods return success bool and take in outputs as pass-by-reference arguments
- OMPL now takes in orientation as an input to setGoal() method

## 15th Dec 2020

### Added

- geometries plugin now supports OOBB for Kinova links and fingers! (Implementation done by using dimensions obtained from mesh files)

### Modified

- geometries plugin now publishes collision markers for Rviz and optionally Kinova collision markers (set by ROS param)
- state validity uses 3 fingers for collision detection rather than merging finger 2 and 3
- state validity uses OOBB for effector link and fingers

### Removed

- geometries_visualiser package (functionality moved to geometries plugin)

## 12th Dec 2020

### Added

- Created package to visualise collision geometries (geometries_visualiser)

### Modified

- Kinova now reverses the path it took to return to home

## 8th Dec 2020

### Added

- openGripper() and closeGripper() methods to controller.cpp

### Modified

- GeometriesPlugin returns geometry_msgs/Pose for centre and bounding box orientation instead of geometry_msgs/Vec3f for just centre
- GeometriesPlugin now returns an OOBB if object collision geometry is a box shape, otherwise returns an AABB
- Set OMPL motion validity resolution to 0.001
- Kinova fingers' collisions now modelled as AABB with increased dimensions to enforce some clearance
- state_validity.h updated and integrated in planner.cpp

### Removed

- isStateValid() removed from planner.cpp
- getGripperGoal() removed from controller.cpp

## 29th Nov 2020

### Added

- Gazebo plugin to obtain desired collision geometries given a model/link name using a ROS service
- Obstacle avoidance using FCL and max reach state validity method in Planner class
- Manipulator name and collision geometries vector to Planner member variables and methods to set them
- Include util.h, Eigen geometry, and more optimal planner headers in planner.h
- Automatic path display when saving path
- Path simplifier and interpolation
- Extra debug prints to Manipulator class
- Method to obtain all collision geometries in Gazebo world and store them in a member variable for Controller class
- Argument for target object as well as a check to allow user to not execute the program if target name is wrong
- Publisher for rviz markers to visualise desired goal and OMPL path waypoints
- Extra status prints to Controller class
- Method to set target name in Controller Class
- Created custom struct in util.h to store collision geometry data
- Two Gazebo worlds (another table world and a bookshelf world)

### Modified

- Set SE3 state space bounds to min: {-1, -1, 0} and max: {1, 1, 0.9}
- End effector start orientation included in OMPL start state
- Moved some headers and typedefs from util.h to controller.h
- simple_traj_node.cpp headers changed due to changes to util.h
- Changed init poses for j2s6s300 and j2n6s300 (shoulder tilts back and elbow is horizontal relative to the ground plane)
- Manipulator FK method returns std::pair composed of Eigen::Vector3d and Eigen::Quaterniond
- Manipulator IK method takes in previous joint states and desired orientation as parameters along with the desired position
- Manipulator IK desired end effector orientation is locked to {-0.5, -0.5, 0.5, 0.5} (quaternion) for the time being
- Minor changes to debug prints format for Manipulator class
- Change inputs to OMPL start and goal starts to use eigen vector3d and quaternion
- Use goToInit() method rather than goToHome() due to fewer collisions, will be replaced by a OMPL method to incorporate collision avoidance
- getJointGoal() ignores first state in OMPL path and passes previously obtained joint states and quaternion orientation to manipulator IK method
- Gripper open position set to {0.4, 0.4, 0.4} rather than {0.0, 0.0, 0.0}
- goToInit() checks if problem solved or not to open/close gripper
- getGripperGoal() sets only one point rather than multiple (method will be removed/changed eventually)
- Re-enable Gazebo grasp plugin
- Add world argument for gazebo.launch
- Gazebo worlds now include collision geometries plugin
- Reduce P gain for j2s6s300 and j2n6s300 joint 6

### Removed

- StateValidity class from planner.cpp (might be added again later, so header is still included)
- Removed DH parameters FK method 