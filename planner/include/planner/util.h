#pragma once

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/action_client.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalID.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Vector3.h>
#include <cmath>
#include <iostream>
#include <chrono>
#include <gazebo_msgs/ModelStates.h>
#include <boost/shared_ptr.hpp>
#include "defines.h"

typedef std::chrono::high_resolution_clock HighResClk;
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ArmActionSimple;
typedef boost::shared_ptr<ArmActionSimple> ArmControlPtr;

namespace chrono = std::chrono;

namespace util
{
    typedef struct CollisionGeometry
    {
        std::string name;
        geometry_msgs::Vector3 min;
        geometry_msgs::Vector3 max;
        geometry_msgs::Vector3 centre;
        geometry_msgs::Vector3 dimension;
    } CollisionGeometry;
}