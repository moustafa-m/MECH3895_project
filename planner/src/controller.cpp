#include "planner/controller.h"

Controller::Controller(ros::NodeHandle* nh)
    : nh_(*nh), manipulator_(nh), planner_(nh, &manipulator_)
{
    this->init();
    this->goToInit();
    this->openGripper();
}

Controller::~Controller()
{

}

void Controller::sendAction(trajectory_msgs::JointTrajectory joint_traj)
{
    control_msgs::FollowJointTrajectoryGoal joint_goal;
    joint_goal.trajectory = joint_traj;
    
    ROS_INFO("%sSending tracjectory actions...", CYAN);
    joint_goal.trajectory.header.stamp = ros::Time::now();
    armAction_->sendGoal(joint_goal);
    armAction_->waitForResult();

    ROS_INFO("%sTrajectories complete!", GREEN);
}

void Controller::openGripper()
{
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

    ROS_INFO("%sGripper opened!", GREEN);
}

void Controller::closeGripper()
{
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

    ROS_INFO("%sGripper closed!", GREEN);
}

void Controller::goToHome()
{
    ROS_INFO("%sMoving to home position...", CYAN);
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
    ROS_INFO("%sDone, Kinova is at home position!", GREEN);
}

void Controller::goToInit()
{
    ROS_INFO("%sMoving to init position...", CYAN);
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

    ROS_INFO("%sDone, Kinova is at init position!", GREEN);
}

void Controller::init()
{
    states_sub_ = nh_.subscribe("/gazebo/model_states", 10, &Controller::statesCallback, this);
    home_srv_ = nh_.advertiseService("go_to_home", &Controller::homeSrvCallback, this);
    init_srv_ = nh_.advertiseService("go_to_init", &Controller::initSrvCallback, this);
    start_plan_srv = nh_.advertiseService("start_plan", &Controller::startPlanSrvCallback, this);
    collisions_client_ = nh_.serviceClient<gazebo_geometries_plugin::geometry>("/gazebo/get_geometry");

    while (states_sub_.getNumPublishers() == 0 && ros::ok())
    {
        ros::spinOnce();
        ROS_INFO_THROTTLE(5, "%sWaiting for Gazebo simulation to come up...", CYAN);
    }
    armAction_.reset(new ArmActionSimple(manipulator_.getName() + "/effort_joint_trajectory_controller/follow_joint_trajectory"));
    gripperAction_.reset(new ArmActionSimple(manipulator_.getName() + "/effort_finger_trajectory_controller/follow_joint_trajectory"));

    armAction_->waitForServer();
    ROS_INFO("%sAll topics and servers up!", GREEN);
}

void Controller::getCollisionBoxes()
{
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
    ROS_INFO("%s[Controller]: Recieved request for %s!", CYAN, req.target.c_str());
    if (states_.name.empty())
    {
        ROS_ERROR("[Controller]: No states received from Gazebo!");
        res.message = "Unable to start, No states received from Gazebo!";
        return true;
    }

    collision_geometries_.clear();
    this->getCollisionBoxes();
    Eigen::Vector3d goal;
    bool found_target = false;
    for (int i = 0; i < collision_geometries_.size(); i++)
    {
        if (collision_geometries_[i].name.compare(req.target+"_collision") == 0)
        {
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

    planner_.clearMarkers();
    
    std::vector<Eigen::Vector3d> positions; std::vector<Eigen::Quaterniond> orientations;
    manipulator_.solveFK(positions, orientations);
    
    planner_.setCollisionGeometries(collision_geometries_);
    planner_.setStart(positions.back(), orientations.back());
    Eigen::Quaterniond q; q.setIdentity();q = orientations.back();
    // q = Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX());
    // goal[0] = -0.00; goal[1] = 0.0; goal[2] = 0.8;
    planner_.setGoal(goal, orientations.back());
    
    trajectory_msgs::JointTrajectory traj;
    if (planner_.plan(traj))
    {
        // planner_.savePath();

        this->openGripper();
        this->sendAction(traj);
        this->closeGripper();
        
        // pause for 1 second to allow time for grasp plugin to attach object
        ros::Duration(1).sleep();

        traj.points.pop_back();
        std::reverse(traj.points.begin(), traj.points.end());
        trajectory_msgs::JointTrajectoryPoint point;
        point.velocities = std::vector<double>(manipulator_.getNumJoints(), 0);
        point.positions = manipulator_.getInitPose();
        traj.points.push_back(point);
        
        for (size_t i = 0; i < traj.points.size(); i++) { traj.points[i].time_from_start = ros::Duration(5.0+(i*2)); }
        this->sendAction(traj);
    }
    else
    {
        ROS_ERROR("Unable to find solution");
        res.message = "Unable to find solution!";
    }
    return true;
}

bool Controller::homeSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    this->goToHome();
    return true;
}

bool Controller::initSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    this->goToInit();
    return true;
}