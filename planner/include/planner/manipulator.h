#pragma once

#include <ros/ros.h>
#include <ros/package.h>
#include <trac_ik/trac_ik.hpp>
#include <sensor_msgs/JointState.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>
#include "defines.h"

class Manipulator
{
public:
    typedef struct DHParameters
    {
        std::vector<double> theta;
        std::vector<double> alpha = {M_PI_2, M_PI, M_PI_2, M_PI/3, M_PI/3, M_PI};
        std::vector<double> a_i = {0, 0.41, 0, 0, 0, 0};
        std::vector<double> d_i = {0.2755, 0, -0.0098, -0.2672, -0.1199, -0.2199};
    } DHParameters;

public:
    Manipulator(ros::NodeHandle* nh);
    ~Manipulator();

    bool solveIK(std::vector<double>& output, const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation, const std::vector<double>& prev_joints);
    bool solveFK(std::vector<Eigen::Vector3d>& positions, std::vector<Eigen::Quaterniond>& orientations, const std::vector<double>& joints_pos = {});
    std::string getName();
    int getNumJoints();
    sensor_msgs::JointState getJointStates();
    std::vector<std::string> getJointNames();
    std::vector<std::string> getFingerNames();
    std::vector<double> getHomePose();
    std::vector<double> getInitPose();

private:
    void initParams();
    void setJointsInfo();
    void initSolvers();
    void setDefaultPoses();
    void setDHParameters();
    void setDHTheta();
    void jointStatesCallback(sensor_msgs::JointStateConstPtr msg);

    ros::NodeHandle nh_;
    ros::Subscriber joints_sub_;

    Eigen::Matrix3d transform_; // (when using DH parameters) transformation is needed for end effector -> base frame for the 6 & 7 DoF spherical arms

    std::string name_ = "j2s7s300";

    TRAC_IK::TRAC_IK* ik_solver_;
    TRAC_IK::SolveType solve_type_;
    double timeout_;

    KDL::ChainFkSolverPos_recursive* fk_solver_;
    KDL::Chain chain_;
    KDL::JntArray kdl_lower_b_, kdl_upper_b_;

    sensor_msgs::JointState joint_states_;

    int num_joints_;

    DHParameters dh_para_;

    std::vector<double> home_pose_;
    std::vector<double> init_pose_;

    std::vector<std::string> joint_names_;
    std::vector<std::string> finger_names_;
};