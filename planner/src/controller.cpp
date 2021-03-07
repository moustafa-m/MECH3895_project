#include "planner/controller.h"

Controller::Controller(ros::NodeHandle* nh, Manipulator* manip)
    : nh_(*nh), manipulator_(manip)
{
    this->init();
    gripper_open_ = false;
}

Controller::~Controller()
{

}

void Controller::sendAction(trajectory_msgs::JointTrajectory joint_traj)
{
    control_msgs::FollowJointTrajectoryGoal joint_goal;
    joint_goal.trajectory = joint_traj;
    
    ROS_INFO("%s[CONTROLLER]: Sending tracjectory actions...", CYAN);
    joint_goal.trajectory.header.stamp = ros::Time::now();
    armAction_->sendGoal(joint_goal);
    armAction_->waitForResult();

    ROS_INFO("%s[CONTROLLER]: Trajectories complete!", GREEN);
}

void Controller::openGripper()
{
    if (gripper_open_)
    {
        // ROS_INFO("%s[CONTROLLER]: Gripper is already open!", GREEN);
        return;
    }

    trajectory_msgs::JointTrajectory msg;
    msg.joint_names.resize(3);
    msg.points.resize(1);

    msg.joint_names = manipulator_->getFingerNames();
    msg.points[0].positions.resize(3);
    msg.points[0].positions = {0.4, 0.4, 0.4};
    msg.points[0].time_from_start = ros::Duration(2);

    control_msgs::FollowJointTrajectoryGoal joint_goal, gripper_goal;
    gripper_goal.trajectory = msg;
    gripper_goal.trajectory.header.stamp = ros::Time::now();
    gripperAction_->sendGoal(gripper_goal);
    gripperAction_->waitForResult();

    gripper_open_ = true;
    ROS_INFO("%s[CONTROLLER]: Gripper opened!", GREEN);
}

void Controller::closeGripper()
{
    if (!gripper_open_)
    {
        // ROS_INFO("%s[CONTROLLER]: Gripper is already closed!", GREEN);
        return;
    }

    trajectory_msgs::JointTrajectory msg;
    msg.joint_names.resize(3);
    msg.points.resize(1);

    msg.joint_names = manipulator_->getFingerNames();
    msg.points[0].positions.resize(3);
    msg.points[0].positions = {0.95, 0.95, 0.95};
    msg.points[0].time_from_start = ros::Duration(2);

    control_msgs::FollowJointTrajectoryGoal joint_goal, gripper_goal;
    gripper_goal.trajectory = msg;
    gripper_goal.trajectory.header.stamp = ros::Time::now();
    gripperAction_->sendGoal(gripper_goal);
    gripperAction_->waitForResult();

    gripper_open_ = false;
    ROS_INFO("%s[CONTROLLER]: Gripper closed!", GREEN);
}

void Controller::goToHome()
{
    std::vector<double> current_pos = manipulator_->getJointStates().position;
    for (int i = 0; i < manipulator_->getFingerNames().size(); i++) current_pos.pop_back();

    if (util::approxEqual(current_pos, manipulator_->getHomePose(), 1e-2))
    {
        // ROS_INFO("%s[CONTROLLER]: Kinova is already at home pose!", GREEN);
        return;
    }

    // ROS_INFO("%s[CONTROLLER]: Moving to home position...", CYAN);
    trajectory_msgs::JointTrajectory joints_msg;
    joints_msg.header.seq += 1;

    // joints
    joints_msg.points.resize(1);
    joints_msg.joint_names.resize(manipulator_->getNumJoints());
    joints_msg.points[0].positions.resize(manipulator_->getNumJoints());

    joints_msg.joint_names = manipulator_->getJointNames();
    joints_msg.points[0].positions = manipulator_->getHomePose();
    joints_msg.points[0].time_from_start = ros::Duration(10);

    joints_msg.header.stamp = ros::Time::now();

    this->sendAction(joints_msg);
    // ROS_INFO("%s[CONTROLLER]: Done, Kinova is at home position!", GREEN);
}

void Controller::goToInit()
{
    std::vector<double> current_pos = manipulator_->getJointStates().position;
    for (int i = 0; i < manipulator_->getFingerNames().size(); i++) current_pos.pop_back();
    
    if (util::approxEqual(current_pos, manipulator_->getInitPose(), 1e-2))
    {
        // ROS_INFO("%s[CONTROLLER]: Kinova is already at init pose!", GREEN);
        return;
    }

    ROS_INFO("%s[CONTROLLER]: Moving to init position...", CYAN);
    trajectory_msgs::JointTrajectory joints_msg;
    joints_msg.header.seq += 1;

    // joints
    joints_msg.points.resize(1); 
    joints_msg.joint_names.resize(manipulator_->getNumJoints());
    joints_msg.points[0].positions.resize(manipulator_->getNumJoints());

    joints_msg.joint_names = manipulator_->getJointNames();    
    joints_msg.points[0].positions = manipulator_->getInitPose();
    joints_msg.points[0].time_from_start = ros::Duration(10);

    joints_msg.header.stamp = ros::Time::now();

    this->sendAction(joints_msg);

    // ROS_INFO("%s[CONTROLLER]: Done, Kinova is at init position!", GREEN);
}

void Controller::init()
{
    home_srv_ = nh_.advertiseService("go_to_home", &Controller::homeSrvCallback, this);
    init_srv_ = nh_.advertiseService("go_to_init", &Controller::initSrvCallback, this);
    open_gripper_srv_ = nh_.advertiseService("open_gripper", &Controller::openGripperSrvCallback, this);
    close_gripper_srv_ = nh_.advertiseService("close_gripper", &Controller::closeGripperSrvCallback, this);

    armAction_.reset(new ArmActionSimple(manipulator_->getName() + "/effort_joint_trajectory_controller/follow_joint_trajectory"));
    gripperAction_.reset(new ArmActionSimple(manipulator_->getName() + "/effort_finger_trajectory_controller/follow_joint_trajectory"));

    armAction_->waitForServer();
    ROS_INFO("%s[CONTROLLER]: All topics and servers up!", GREEN);
}

bool Controller::homeSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    ROS_INFO("%s[CONTROLLER]: Recieved srv request to go to home", CYAN);
    this->goToHome();
    return true;
}

bool Controller::initSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    ROS_INFO("%s[CONTROLLER]: Recieved srv request to go to init pose",  CYAN);
    this->goToInit();
    return true;
}

bool Controller::openGripperSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    ROS_INFO("%s[CONTROLLER]: Recieved srv request to open gripper",  CYAN);
    this->openGripper();
    return true;
}

bool Controller::closeGripperSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    ROS_INFO("%s[CONTROLLER]: Recieved srv request to close gripper",  CYAN);
    this->closeGripper();
    return true;
}