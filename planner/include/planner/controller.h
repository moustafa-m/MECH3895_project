#pragma once

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <gazebo_msgs/ModelStates.h>
#include <planner/start_plan.h>
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
    void sendAction(trajectory_msgs::JointTrajectory joint_traj);
    void openGripper();
    void closeGripper();
    void goToHome();
    void goToInit();
    
private:
    void init();
    void getCollisionBoxes();
    void statesCallback(gazebo_msgs::ModelStatesConstPtr msg);
    bool startPlanSrvCallback(planner::start_plan::Request& req, planner::start_plan::Response& res);
    bool homeSrvCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    bool initSrvCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    bool openGripperSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool closeGripperSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

    Manipulator manipulator_;
    Planner planner_;

    ArmControlPtr armAction_;
    ArmControlPtr gripperAction_;

    ros::NodeHandle nh_;
    ros::Subscriber states_sub_;
    ros::ServiceServer start_plan_srv;
    ros::ServiceServer home_srv_;
    ros::ServiceServer init_srv_;
    ros::ServiceServer open_gripper_srv_;
    ros::ServiceServer close_gripper_srv_;
    ros::ServiceClient collisions_client_;

    gazebo_msgs::ModelStates states_;
    std::vector<util::CollisionGeometry> collision_geometries_;
    
    bool gripper_open_;
};