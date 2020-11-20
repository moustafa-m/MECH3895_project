#include "planner/util.h"

std::vector<double> home_pose = {0.0, 2.9, 1.3, 4.2, 1.4, 0.0};
std::vector<double> init_pose = {0.0, M_PI, 0.4, 2.4, 1.4, 0};

void statesCallback(const gazebo_msgs::ModelStatesConstPtr& msg);
trajectory_msgs::JointTrajectory getJointGoal();
trajectory_msgs::JointTrajectory getGripperGoal();

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_traj_node");

    ros::NodeHandle nh;
    ros::Rate rate(10);

    std::string robot_name;
    ros::param::param<std::string>("~robot", robot_name, "j2n6s300");

    ROS_INFO_STREAM("Robot name: " << robot_name);
    ArmActionSimple armAction(robot_name + "/effort_joint_trajectory_controller/follow_joint_trajectory");
    ArmActionSimple gripperAction(robot_name + "/effort_finger_trajectory_controller/follow_joint_trajectory");
    ros::Subscriber states_sub = nh.subscribe("/gazebo/model_states", 10, &statesCallback);

    int i = 0;
    while (!armAction.waitForServer(ros::Duration(2.0)) && ros::ok() && i <= 2) { ROS_INFO("Connecting to Action server..."); }
    if (!armAction.isServerConnected()) { throw std::runtime_error("Action server not connected!"); }

    ROS_INFO("Starting node...");

    bool sent = false;
    while (ros::ok())
    {
        ROS_INFO_ONCE("%sNode started!", GREEN);
        control_msgs::FollowJointTrajectoryGoal goal, gripper_goal;
        if (!sent)
        {
            goal.trajectory = getJointGoal();
            gripper_goal.trajectory = getGripperGoal();
            armAction.sendGoal(goal);
            gripperAction.sendGoal(gripper_goal);
            sent = true;
        }
        
        // ros::Duration(4).sleep();
        // ROS_INFO("breaking!");

        // goal.trajectory.header.stamp = gripper_goal.trajectory.header.stamp = ros::Time::now();
        // goal.trajectory.points[0].positions = home_pose;
        // gripper_goal.trajectory.points[0].positions = {1, 1, 1};
        // armAction.sendGoal(goal);
        // gripperAction.sendGoal(gripper_goal);
        
        ros::spinOnce();
        rate.sleep();
    }
    
    std::cout << CYAN << "Node shutdown!" << NC << std::endl;

    return EXIT_SUCCESS;
}

void statesCallback(const gazebo_msgs::ModelStatesConstPtr& msg)
{
    std::stringstream ss;
    for (int i = 0; i < msg->name.size(); i++)
    {
        ss << "Model: " << msg->name[i] << ", Pose: " << msg->pose[i] << "\n";
    }
    // ROS_INFO_STREAM_THROTTLE(5, ss.str());
}

trajectory_msgs::JointTrajectory getJointGoal()
{
    trajectory_msgs::JointTrajectory msg;
    msg.header.stamp = ros::Time::now();
    msg.joint_names.resize(6);

    msg.joint_names = {"j2n6s300_joint_1", "j2n6s300_joint_2", "j2n6s300_joint_3", "j2n6s300_joint_4", "j2n6s300_joint_5", "j2n6s300_joint_6"};

    msg.points.resize(1);
    msg.points[0].positions.resize(6);
    msg.points[0].effort.resize(6);
    msg.points[0].velocities.resize(6);

    msg.points[0].effort[0] = msg.points[0].velocities[0] = 0.0;
    msg.points[0].effort[1] = msg.points[0].velocities[1] = 0.0;
    msg.points[0].effort[2] = msg.points[0].velocities[2] = 0.0;
    msg.points[0].effort[3] = msg.points[0].velocities[3] = 0.0;
    msg.points[0].effort[4] = msg.points[0].velocities[4] = 0.0;
    msg.points[0].effort[5] = msg.points[0].velocities[5] = 0.0;

    msg.points[0].positions[0] = 0.0;
    msg.points[0].positions[1] = M_PI;
    msg.points[0].positions[2] = M_PI;
    msg.points[0].positions[3] = M_PI;
    msg.points[0].positions[4] = M_PI;
    msg.points[0].positions[5] = 0;

    msg.points[0].time_from_start = ros::Duration(10);

    return msg;
}

trajectory_msgs::JointTrajectory getGripperGoal()
{
    trajectory_msgs::JointTrajectory msg;
    msg.header.stamp = ros::Time::now();
    msg.joint_names.resize(3); msg.joint_names = {"j2n6s300_joint_finger_1", "j2n6s300_joint_finger_2", "j2n6s300_joint_finger_3"};
    msg.points.resize(1); msg.points[0].positions.resize(3); msg.points[0].velocities.resize(3); msg.points[0].effort.resize(3);
    msg.points[0].positions[0] = 0.5;
    msg.points[0].positions[1] = 0.75*msg.points[0].positions[0];
    msg.points[0].positions[2] = 1*msg.points[0].positions[0];

    msg.points[0].effort[0] = msg.points[0].velocities[0] = 0.0;
    msg.points[0].effort[1] = msg.points[0].velocities[1] = 0.0;
    msg.points[0].effort[2] = msg.points[0].velocities[2] = 0.0;

    msg.points[0].time_from_start = ros::Duration(2);

    return msg;
}