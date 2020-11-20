#include "planner/controller.h"

Controller::Controller(ros::NodeHandle* nh)
    : nh_(*nh), manipulator_(nh)
{
    this->init();
    goToInit();
}

Controller::~Controller()
{

}

void Controller::run()
{
    if (states_.pose.empty() || solved_) return;

    Eigen::Vector3d goal;
    for (int i = 0; i < states_.name.size(); i++)
    {
        if (states_.name[i].compare("coke_can") == 0)
        {
            goal[0] = states_.pose[i].position.x;
            goal[1] = states_.pose[i].position.y;
            goal[2] = states_.pose[i].position.z + 0.1;
            break;
        }
    }
    // goal[0] = -0.4; goal[1] = 0; goal[2] = 0.8;
    std::vector<Eigen::Vector3d> start_pose = manipulator_.solveFK();
    planner_.setStart(start_pose[0]); planner_.setGoal(goal);
    
    og::PathGeometric solution = planner_.plan();
    
    if (solution.check())
    {
        ROS_INFO("%sObtaining joint angles...", CYAN);
        // getJointGoal(solution);
        sendAction(getJointGoal(solution), getGripperGoal(solution));
        solved_ = true;

        goToHome();
        ros::shutdown();
    }
}

trajectory_msgs::JointTrajectory Controller::getJointGoal(og::PathGeometric& path)
{
    std::vector<ob::State*> path_states = path.getStates();

    int num_joints = manipulator_.getNumJoints();
    trajectory_msgs::JointTrajectory msg;
    msg.joint_names.resize(num_joints);
    msg.points.resize(path_states.size());

    msg.joint_names = manipulator_.getJointNames();

    msg.points[0].positions = manipulator_.getInitPose();
    msg.points[0].time_from_start = ros::Duration(1);

    for (size_t i = 1; i < path_states.size(); i++)
    {
        msg.points[i].positions.resize(num_joints);
        msg.points[i].effort.resize(num_joints);
        msg.points[i].velocities.resize(num_joints);
        msg.points[i].accelerations.resize(num_joints);

        Eigen::Vector3d position;
        position << path_states[i]->as<ob::SE3StateSpace::StateType>()->getX(),
                path_states[i]->as<ob::SE3StateSpace::StateType>()->getY(),
                path_states[i]->as<ob::SE3StateSpace::StateType>()->getZ();

        std::vector<double> angles = manipulator_.solveIK(position);
        if (angles.empty())
        {
            ROS_ERROR("No angles obtained from IK!");
            exit(-1);
        }

        msg.points[i].positions = angles;
        for (int j = 0; j < num_joints; j++)
        {
            msg.points[i].effort[j] = 3;
            msg.points[i].velocities[j] = 0;
            msg.points[i].accelerations[j] = 0;
        }
        msg.points[i].time_from_start = ros::Duration(10+i);
    }

    return msg;
}

trajectory_msgs::JointTrajectory Controller::getGripperGoal(og::PathGeometric& path)
{
    std::vector<ob::State*> path_states = path.getStates();

    int num_joints = manipulator_.getNumJoints();
    trajectory_msgs::JointTrajectory msg;
    msg.joint_names.resize(3);
    msg.points.resize(path_states.size());

    msg.joint_names = manipulator_.getFingerNames();

    for (size_t i = 0; i < path_states.size(); i++)
    {
        msg.points[i].positions.resize(3);
        msg.points[i].effort.resize(3);
        msg.points[i].velocities.resize(3);
        msg.points[i].accelerations.resize(3);
        
        if (i == path_states.size() - 1)
        {
            msg.points[i].positions = {0.95, 0.95, 0.95};
            msg.points[i].effort = {2000, 2000, 2000};
        }

        msg.points[i].velocities = {0, 0, 0};
        msg.points[i].accelerations = {0, 0, 0};
        
        msg.points[i].time_from_start = ros::Duration(2+i);
    }

    return msg;
}

void Controller::goToHome()
{
    ROS_INFO("%sMoving to home position...", CYAN);
    trajectory_msgs::JointTrajectory joints_msg, gripper_msg;
    joints_msg.header.seq = gripper_msg.header.seq += 1;
    joints_msg.header.stamp = gripper_msg.header.stamp = ros::Time::now();

    // joints
    joints_msg.joint_names.resize(manipulator_.getNumJoints()); joints_msg.points.resize(1); joints_msg.points[0].positions.resize(6);
    joints_msg.points[0].effort.resize(manipulator_.getNumJoints());
    joints_msg.joint_names = manipulator_.getJointNames();
    joints_msg.points[0].positions = manipulator_.getHomePose();
    for (int i = 0; i < manipulator_.getNumJoints(); i++) { joints_msg.points[0].effort[i] = 1; }
    joints_msg.points[0].time_from_start = ros::Duration(10);

    // gripper
    gripper_msg.joint_names.resize(3); gripper_msg.points.resize(1); gripper_msg.points[0].positions.resize(3);
    gripper_msg.points[0].effort.resize(3);
    gripper_msg.joint_names = manipulator_.getFingerNames();
    gripper_msg.points[0].positions = {0.95, 0.95, 0.95};
    gripper_msg.points[0].effort = {5, 5, 5};
    gripper_msg.points[0].time_from_start = ros::Duration(3);

    sendAction(joints_msg, gripper_msg);
    ROS_INFO("%sDone!", GREEN);
}

void Controller::goToInit()
{
    ROS_INFO("%sMoving to init position...", CYAN);
    trajectory_msgs::JointTrajectory joints_msg, gripper_msg;
    joints_msg.header.seq = gripper_msg.header.seq += 1;
    joints_msg.header.stamp = gripper_msg.header.stamp = ros::Time::now();

    // joints
    joints_msg.joint_names.resize(manipulator_.getNumJoints()); joints_msg.points.resize(1); joints_msg.points[0].positions.resize(6);
    joints_msg.points[0].effort.resize(manipulator_.getNumJoints());
    joints_msg.joint_names = manipulator_.getJointNames();    
    joints_msg.points[0].positions = manipulator_.getInitPose();
    for (int i = 0; i < manipulator_.getNumJoints(); i++) { joints_msg.points[0].effort[i] = 1; }
    joints_msg.points[0].time_from_start = ros::Duration(10);
    
    // gripper
    gripper_msg.joint_names.resize(3); gripper_msg.points.resize(1); gripper_msg.points[0].positions.resize(3);
    gripper_msg.joint_names = manipulator_.getFingerNames();
    gripper_msg.points[0].positions = {0, 0, 0};
    gripper_msg.points[0].time_from_start = ros::Duration(3);

    joints_msg.header.stamp = gripper_msg.header.stamp = ros::Time::now();

    sendAction(joints_msg, gripper_msg);
    ROS_INFO("%sDone!", GREEN);
}

void Controller::init()
{
    states_sub_ = nh_.subscribe("/gazebo/model_states", 10, &Controller::statesCallback, this);
    home_srv_ = nh_.advertiseService("go_to_home", &Controller::homeSrvCallback, this);
    init_srv_ = nh_.advertiseService("go_to_init", &Controller::initSrvCallback, this);

    while (states_sub_.getNumPublishers() == 0 && ros::ok())
    {
        ros::spinOnce();
        ROS_INFO_THROTTLE(5, "%sWaiting for Gazebo simulation to come up...", CYAN);
    }
    armAction_.reset(new ArmActionSimple(manipulator_.getName() + "/effort_joint_trajectory_controller/follow_joint_trajectory"));
    gripperAction_.reset(new ArmActionSimple(manipulator_.getName() + "/effort_finger_trajectory_controller/follow_joint_trajectory"));

    armAction_->waitForServer();
    ROS_INFO("%sAll topics and servers up!", GREEN);

    // timer_ = nh_.createTimer(ros::Duration(1/FREQ), &Controller::timerCallback, this);
}

void Controller::sendAction(trajectory_msgs::JointTrajectory joint_traj, trajectory_msgs::JointTrajectory gripper_traj)
{
    control_msgs::FollowJointTrajectoryGoal joint_goal, gripper_goal;
    joint_goal.trajectory = joint_traj;
    gripper_goal.trajectory = gripper_traj;
    
    joint_goal.trajectory.header.stamp = ros::Time::now();
    armAction_->sendGoal(joint_goal);
    armAction_->waitForResult();

    gripper_goal.trajectory.header.stamp = ros::Time::now();
    gripperAction_->sendGoal(gripper_goal);
    gripperAction_->waitForResult();
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
    ros::init(argc, argv, "controller_test_node");
    ros::NodeHandle nh;
    std::cout << CYAN << "Starting test run!" << NC << std::endl;

    Controller controller(&nh);
    while (ros::ok())
    {
        ros::spinOnce();
        controller.run();
    }

    std::cout << GREEN << "Run complete, exiting!" << NC << std::endl;
}