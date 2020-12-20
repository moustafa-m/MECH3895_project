#include "planner/controller.h"

Controller::Controller(ros::NodeHandle* nh)
    : nh_(*nh), manipulator_(nh)
{
    this->init();
    this->goToInit();
}

Controller::~Controller()
{

}

void Controller::run()
{
    if (states_.pose.empty() || solved_) return;

    // clear all rviz markers
    visualization_msgs::Marker marker;
	marker.action = visualization_msgs::Marker::DELETEALL;
    marker_pub_.publish(marker);

    this->getCollisionBoxes();
    Eigen::Vector3d goal;
    bool found_target = false;
    for (int i = 0; i < collision_geometries_.size(); i++)
    {
        if (collision_geometries_[i].name.find(target_) != std::string::npos)
        {
            goal << collision_geometries_[i].pose.position.x,
                    collision_geometries_[i].pose.position.y,
                    collision_geometries_[i].pose.position.z;
            found_target = true;
            break;
        }
    }
    
    if (!found_target) { ROS_FATAL("Unable to find collision geometry for [%s]", target_.c_str()); exit(-1); }

    // ----> Marking goal
    marker.id = 0;
    marker.scale.x = marker.scale.y = 0.02;
    marker.color.b = marker.color.a = 1.0;
    marker.color.r = marker.color.g = 0.0;
    marker.ns = "goal";
    marker.header.frame_id = manipulator_.getName() + "_link_base";
    marker.header.stamp = ros::Time::now();
    geometry_msgs::Point p;
    p.x = goal[0]; p.y = p.y = goal[1]; p.z = goal[2]; marker.points.push_back(p);
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::POINTS;
    marker_pub_.publish(marker);
    // <----
    
    std::vector<Eigen::Vector3d> positions; std::vector<Eigen::Quaterniond> orientations;
    manipulator_.solveFK(positions, orientations);
    
    planner_.setCollisionGeometries(collision_geometries_);
    planner_.setManipulatorName(manipulator_.getName());
    planner_.setStart(positions.back(), orientations.back());
    planner_.setGoal(goal, orientations.back(), target_);
    
    og::PathGeometric solution = planner_.plan();
    
    if (solution.check())
    {
        planner_.savePath();
        for (size_t i = 0; i < solution.getStateCount(); i++)
        {
            marker.id = i;
            marker.action = visualization_msgs::Marker::ADD;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.scale.x = 0.05, marker.scale.y = 0.01, marker.scale.z = 0.03;
            marker.color.g = marker.color.r = marker.color.a = 1.0;
            marker.color.b = 0.0;
            marker.ns = "path";
            marker.header.frame_id = manipulator_.getName() + "_link_base";
            marker.header.stamp = ros::Time::now();

            geometry_msgs::Point p;
            p.x = solution.getStates()[i]->as<ob::SE3StateSpace::StateType>()->getX();
            p.y = solution.getStates()[i]->as<ob::SE3StateSpace::StateType>()->getY();
            p.z = solution.getStates()[i]->as<ob::SE3StateSpace::StateType>()->getZ();
            marker.pose.position = p;

            Eigen::Quaterniond quat;
            quat.w() = solution.getStates()[i]->as<ob::SE3StateSpace::StateType>()->rotation().w;
            quat.x() = solution.getStates()[i]->as<ob::SE3StateSpace::StateType>()->rotation().x;
            quat.y() = solution.getStates()[i]->as<ob::SE3StateSpace::StateType>()->rotation().y;
            quat.z() = solution.getStates()[i]->as<ob::SE3StateSpace::StateType>()->rotation().z;
            // arrow is parallel to x-axis when rotation is identity but kinova end effector is parallel
            // to z-axis when at identity, so transform is needed
            quat = quat * Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitY());

            marker.pose.orientation.w = quat.w();
            marker.pose.orientation.x = quat.x();
            marker.pose.orientation.y = quat.y();
            marker.pose.orientation.z = quat.z();
            marker.points.resize(0);

            marker_pub_.publish(marker);
        }
        ROS_INFO_STREAM(CYAN << "Obtaining joint angles for " << solution.getStateCount() << " states...");
        
        trajectory_msgs::JointTrajectory traj = this->getJointGoal(solution);
        this->sendAction(traj);
        this->closeGripper();
        solved_ = true;

        traj.points.pop_back();
        std::reverse(traj.points.begin(), traj.points.end());
        trajectory_msgs::JointTrajectoryPoint point;
        point.velocities = std::vector<double>(7, 0);
        point.positions = manipulator_.getInitPose();
        traj.points.push_back(point);
        
        for (size_t i = 0; i < traj.points.size(); i++) { traj.points[i].time_from_start = ros::Duration(5.0+(i*2)); }
        this->sendAction(traj);
        ros::shutdown();
    }
}

trajectory_msgs::JointTrajectory Controller::getJointGoal(og::PathGeometric& path)
{
    std::vector<ob::State*> path_states = path.getStates();
    path_states.erase(path_states.begin());

    int num_joints = manipulator_.getNumJoints();
    trajectory_msgs::JointTrajectory msg;
    msg.joint_names.resize(num_joints);
    msg.points.resize(path_states.size());

    msg.joint_names = manipulator_.getJointNames();

    for (size_t i = 0; i < path_states.size(); i++)
    {
        msg.points[i].positions.resize(num_joints);
        msg.points[i].effort.resize(num_joints);
        msg.points[i].velocities.resize(num_joints);
        msg.points[i].accelerations.resize(num_joints);

        Eigen::Vector3d position;
        position << path_states[i]->as<ob::SE3StateSpace::StateType>()->getX(),
                path_states[i]->as<ob::SE3StateSpace::StateType>()->getY(),
                path_states[i]->as<ob::SE3StateSpace::StateType>()->getZ();

        Eigen::Quaterniond orientation;
        orientation.w() = path_states[i]->as<ob::SE3StateSpace::StateType>()->rotation().w;
        orientation.x() = path_states[i]->as<ob::SE3StateSpace::StateType>()->rotation().x;
        orientation.y() = path_states[i]->as<ob::SE3StateSpace::StateType>()->rotation().y;
        orientation.z() = path_states[i]->as<ob::SE3StateSpace::StateType>()->rotation().z;

        std::vector<double> angles;
        if (i == 0) { manipulator_.solveIK(angles, position, orientation, manipulator_.getInitPose()); }
        else { manipulator_.solveIK(angles, position, orientation, msg.points[i-1].positions); }

        msg.points[i].positions = angles;
        for (int j = 0; j < num_joints; j++)
        {
            msg.points[i].effort[j] = 1000;
            msg.points[i].velocities[j] = 0;
            msg.points[i].accelerations[j] = 0;
        }
        msg.points[i].time_from_start = ros::Duration(5+(i*2));

        if (!ros::ok()) { exit(0); }
    }

    return msg;
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
    if (!solved_) { this->openGripper(); }

    ROS_INFO("%sDone, Kinova is at init position!", GREEN);
}

void Controller::setTargetName(std::string target)
{
    target_ = target;
}

void Controller::init()
{
    states_sub_ = nh_.subscribe("/gazebo/model_states", 10, &Controller::statesCallback, this);
    home_srv_ = nh_.advertiseService("go_to_home", &Controller::homeSrvCallback, this);
    init_srv_ = nh_.advertiseService("go_to_init", &Controller::initSrvCallback, this);
    collisions_client_ = nh_.serviceClient<gazebo_geometries_plugin::geometry>("/gazebo/get_geometry");
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/visualization_marker", 10);

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

bool Controller::homeSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    goToHome();
    return true;
}

bool Controller::initSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    goToInit();
    return true;
}

void Controller::statesCallback(gazebo_msgs::ModelStatesConstPtr msg)
{
    states_ = *msg;
}

int main(int argc, char** argv)
{
    std::string target;
    ros::init(argc, argv, "controller_test_node");
    ros::NodeHandle nh("~");
    nh.getParam("target", target);
    std::cout << CYAN << "Starting test run!" << NC << std::endl;

    if (target.empty()) { target = "coke_can"; }

    std::cout << CYAN << "Target object set to: " << target << "\nInput any key to continue or n to exit: ";
    std::string in;
    std::cin >> in;
    if (in.compare("n") == 0)
    {
        std::cout << GREEN << "Exiting!" << NC << std::endl;
        ros::shutdown();
        return 0;
    }

    Controller controller(&nh);
    controller.setTargetName(target);
    while (ros::ok())
    {
        ros::spinOnce();
        controller.run();
    }

    std::cout << GREEN << "Run complete, exiting!" << NC << std::endl;
    return 0;
}