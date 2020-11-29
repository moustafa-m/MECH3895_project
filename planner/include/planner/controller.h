#pragma once

#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <actionlib_msgs/GoalID.h>
#include <actionlib/client/action_client.h>
#include <actionlib/client/simple_action_client.h>
#include <gazebo_geometries_plugin/geometry.h>

#include "util.h"
#include "planner.h"
#include "manipulator.h"

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ArmActionSimple;
typedef boost::shared_ptr<ArmActionSimple> ArmControlPtr;

class Controller
{
public:
    Controller(ros::NodeHandle* nh);
    ~Controller();

    void run();
    trajectory_msgs::JointTrajectory getJointGoal(og::PathGeometric& path);
    trajectory_msgs::JointTrajectory getGripperGoal(og::PathGeometric& path);
    void goToHome();
    void goToInit();
    void setTargetName(std::string target);
    
private:
    void init();
    void sendAction(trajectory_msgs::JointTrajectory joint_traj, trajectory_msgs::JointTrajectory gripper_traj);
    void getCollisionBoxes();
    void statesCallback(gazebo_msgs::ModelStatesConstPtr msg);
    bool homeSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool initSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    
    Planner planner_;
    Manipulator manipulator_;

    ArmControlPtr armAction_;
    ArmControlPtr gripperAction_;

    ros::NodeHandle nh_;
    ros::ServiceServer home_srv_;
    ros::ServiceServer init_srv_;
    ros::Publisher marker_pub_;
    ros::Subscriber states_sub_;
    ros::ServiceClient collisions_client_;

    gazebo_msgs::ModelStates states_;
    std::vector<util::CollisionGeometry> collision_geometries_;
    std::string target_;
    
    bool solved_ = false;
};