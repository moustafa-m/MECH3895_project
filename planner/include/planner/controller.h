#pragma once

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib_msgs/GoalID.h>
#include <actionlib/client/action_client.h>
#include <actionlib/client/simple_action_client.h>

#include "util.h"
#include "manipulator.h"

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ArmActionSimple;
typedef boost::shared_ptr<ArmActionSimple> ArmControlPtr;

class Controller
{
public:
    Controller(ros::NodeHandle* nh, Manipulator* manip);
    ~Controller();

    void sendAction(trajectory_msgs::JointTrajectory joint_traj);
    void openGripper();
    void closeGripper();
    void goToHome();
    void goToInit();
    
private:
    void init();
    bool homeSrvCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    bool initSrvCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    bool openGripperSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool closeGripperSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

    Manipulator* manipulator_;

    ArmControlPtr armAction_;
    ArmControlPtr gripperAction_;

    ros::NodeHandle nh_;
    ros::ServiceServer home_srv_;
    ros::ServiceServer init_srv_;
    ros::ServiceServer open_gripper_srv_;
    ros::ServiceServer close_gripper_srv_;
    
    bool gripper_open_;
};