# Changelog

Noteable/major changes will be listed here

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