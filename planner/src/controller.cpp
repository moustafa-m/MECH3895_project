#include "planner/controller.h"

Controller::Controller(ros::NodeHandle* nh)
    : nh_(*nh), manipulator_(nh), planner_(nh, &manipulator_)
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
        ROS_INFO("%s[CONTROLLER]: Gripper is already open!", GREEN);
        return;
    }

    trajectory_msgs::JointTrajectory msg;
    msg.joint_names.resize(3);
    msg.points.resize(1);

    msg.joint_names = manipulator_.getFingerNames();
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
        ROS_INFO("%s[CONTROLLER]: Gripper is already closed!", GREEN);
        return;
    }

    trajectory_msgs::JointTrajectory msg;
    msg.joint_names.resize(3);
    msg.points.resize(1);

    msg.joint_names = manipulator_.getFingerNames();
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
    std::vector<double> current_pos = manipulator_.getJointStates().position;
    for (int i = 0; i < manipulator_.getFingerNames().size(); i++) current_pos.pop_back();

    if (util::approxEqual(current_pos, manipulator_.getHomePose(), 1e-2))
    {
        ROS_INFO("%s[CONTROLLER]: Kinova is already at home pose!", GREEN);
        return;
    }

    ROS_INFO("%s[CONTROLLER]: Moving to home position...", CYAN);
    trajectory_msgs::JointTrajectory joints_msg;
    joints_msg.header.seq += 1;

    // joints
    joints_msg.points.resize(1);
    joints_msg.joint_names.resize(manipulator_.getNumJoints());
    joints_msg.points[0].positions.resize(manipulator_.getNumJoints());

    joints_msg.joint_names = manipulator_.getJointNames();
    joints_msg.points[0].positions = manipulator_.getHomePose();
    joints_msg.points[0].time_from_start = ros::Duration(10);

    joints_msg.header.stamp = ros::Time::now();

    this->sendAction(joints_msg);
    ROS_INFO("%s[CONTROLLER]: Done, Kinova is at home position!", GREEN);
}

void Controller::goToInit()
{
    std::vector<double> current_pos = manipulator_.getJointStates().position;
    for (int i = 0; i < manipulator_.getFingerNames().size(); i++) current_pos.pop_back();
    
    if (util::approxEqual(current_pos, manipulator_.getInitPose(), 1e-2))
    {
        ROS_INFO("%s[CONTROLLER]: Kinova is already at init pose!", GREEN);
        return;
    }

    ROS_INFO("%s[CONTROLLER]: Moving to init position...", CYAN);
    trajectory_msgs::JointTrajectory joints_msg;
    joints_msg.header.seq += 1;

    // joints
    joints_msg.points.resize(1); 
    joints_msg.joint_names.resize(manipulator_.getNumJoints());
    joints_msg.points[0].positions.resize(manipulator_.getNumJoints());

    joints_msg.joint_names = manipulator_.getJointNames();    
    joints_msg.points[0].positions = manipulator_.getInitPose();
    joints_msg.points[0].time_from_start = ros::Duration(10);

    joints_msg.header.stamp = ros::Time::now();

    this->sendAction(joints_msg);

    ROS_INFO("%s[CONTROLLER]: Done, Kinova is at init position!", GREEN);
}

void Controller::init()
{
    states_sub_ = nh_.subscribe("/gazebo/model_states", 10, &Controller::statesCallback, this);
    start_plan_srv = nh_.advertiseService("start_plan", &Controller::startPlanSrvCallback, this);
    home_srv_ = nh_.advertiseService("go_to_home", &Controller::homeSrvCallback, this);
    init_srv_ = nh_.advertiseService("go_to_init", &Controller::initSrvCallback, this);
    open_gripper_srv_ = nh_.advertiseService("open_gripper", &Controller::openGripperSrvCallback, this);
    close_gripper_srv_ = nh_.advertiseService("close_gripper", &Controller::closeGripperSrvCallback, this);
    collisions_client_ = nh_.serviceClient<gazebo_geometries_plugin::geometry>("/gazebo/get_geometry");

    while (states_sub_.getNumPublishers() == 0 && ros::ok())
    {
        ROS_INFO_THROTTLE(5, "%s[CONTROLLER]: Waiting for Gazebo topics to come up...", CYAN);
    }
    armAction_.reset(new ArmActionSimple(manipulator_.getName() + "/effort_joint_trajectory_controller/follow_joint_trajectory"));
    gripperAction_.reset(new ArmActionSimple(manipulator_.getName() + "/effort_finger_trajectory_controller/follow_joint_trajectory"));

    armAction_->waitForServer();
    ROS_INFO("%s[CONTROLLER]: All topics and servers up!", GREEN);
}

void Controller::getCollisionBoxes()
{
    collision_geometries_.clear();

    for (size_t i = 0; i < states_.name.size(); i++)
    {
        if (states_.name[i].find(manipulator_.getName()) != std::string::npos) continue;

        gazebo_geometries_plugin::geometry srv;
        srv.request.model_name = states_.name[i];
        if (collisions_client_.call(srv))
        {
            for (size_t j = 0; j < srv.response.name.size(); j++)
            {
                util::CollisionGeometry temp;
                temp.name = srv.response.name[j];
                temp.pose = srv.response.pose[j];
                temp.min = srv.response.min_bounds[j];
                temp.max = srv.response.max_bounds[j];
                temp.dimension = srv.response.dimensions[j];
                // std::cout << srv.response.dimensions[j].x << " -- " << srv.response.name[j] << " -- " << states_.name[i] << std::endl;
                collision_geometries_.push_back(temp);
            }
        }
    }

    int num_joints = manipulator_.getNumJoints();
    //  collision geometries for Kinova links and fingers
    for (int i = 0 ; i < num_joints+3; i++)
    {
        std::string link;
        if (i < manipulator_.getNumJoints()) { link = "_link_" + std::to_string(i+1); }
        else { link = "_link_finger_" + std::to_string((i+1)-num_joints); }

        gazebo_geometries_plugin::geometry srv;
        srv.request.model_name = manipulator_.getName() + link;
        if (collisions_client_.call(srv))
        {
            for (size_t j = 0; j < srv.response.name.size(); j++)
            {
                util::CollisionGeometry temp;
                temp.name = srv.response.name[j];
                temp.pose = srv.response.pose[j];
                temp.min = srv.response.min_bounds[j];
                temp.max = srv.response.max_bounds[j];
                temp.dimension = srv.response.dimensions[j];
                collision_geometries_.push_back(temp);
            }
        }
    }

    // reset saved states to force recheck of world models
    states_ = gazebo_msgs::ModelStates();
}

void Controller::statesCallback(gazebo_msgs::ModelStatesConstPtr msg)
{
    states_ = *msg;
}

bool Controller::startPlanSrvCallback(planner::start_plan::Request& req, planner::start_plan::Response& res)
{
    this->goToInit();

    ROS_INFO("%s[Controller]: Recieved request for %s!", CYAN, req.target.c_str());
    if (states_.name.empty())
    {
        ROS_ERROR("[Controller]: No states received from Gazebo!");
        res.message = "Unable to start, No states received from Gazebo!";
        return true;
    }

    if (req.target.find("_collision") == std::string::npos) req.target += "_collision";

    this->openGripper();
    this->getCollisionBoxes();
    Eigen::Vector3d goal;
    bool found_target = false;
    for (int i = 0; i < collision_geometries_.size(); i++)
    {
        if (collision_geometries_[i].name.compare(req.target) == 0)
        {
            planner_.setTargetGeometry(collision_geometries_[i]);
            goal << collision_geometries_[i].pose.position.x,
                    collision_geometries_[i].pose.position.y,
                    collision_geometries_[i].pose.position.z;
            found_target = true;
            break;
        }
    }

    if (!found_target)
    {
        ROS_ERROR("[Controller]: Unable to find collision geometry for [%s]", req.target.c_str());
        res.message = "Unable to start, target not found!";
        return true;
    }
    else if (sqrt((goal[0]*goal[0]) + (goal[1]*goal[1]) + (goal[2]*goal[2])) > 0.95)
    {
        ROS_ERROR("[Controller]: Target is out of reach!");
        res.message = "Unable to start, target is out of reach!";
        return true;
    }

    std::vector<Eigen::Vector3d> start_positions; std::vector<Eigen::Quaterniond> start_orientations;
    manipulator_.solveFK(start_positions, start_orientations);
    
    planner_.clearMarkers();
    planner_.setCollisionGeometries(collision_geometries_);
    planner_.setStart(start_positions.back(), start_orientations.back());
    planner_.setGoal(goal, start_orientations.back());

    trajectory_msgs::JointTrajectory traj;
    if (planner_.plan(traj))
    {
        this->sendAction(traj);
        this->closeGripper();
        
        // pause for 1 second to allow time for grasp plugin to attach object
        ros::Duration(1).sleep();
    }
    else
    {
        ROS_ERROR("[CONTROLLER]: Planner unable to find solution");
        res.message = "Unable to find solution!";
    }
    return true;
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