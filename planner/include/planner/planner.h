#pragma once

#include <fstream>
#include <ros/package.h>
#include <eigen3/Eigen/Geometry>
#include <boost/filesystem.hpp>
#include <visualization_msgs/Marker.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/spaces/SE3StateSpace.h>

#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/PathSimplifier.h>

#include "state_validity.h"
#include "util.h"
#include "defines.h"
#include "manipulator.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

class Planner
{
public:
    Planner(ros::NodeHandle* nh, Manipulator* manip);
    ~Planner();

    bool plan(trajectory_msgs::JointTrajectory& traj);
    void setStart(const Eigen::Vector3d& start, const Eigen::Quaterniond& orientation);
    void setGoal(const Eigen::Vector3d& goal, const Eigen::Quaterniond& orientation);
    void setCollisionGeometries(const std::vector<util::CollisionGeometry>& collision_boxes);
    void setTargetGeometry(util::CollisionGeometry geom);
    void savePath();
    void clearMarkers();

private:
    void init();
    void initROS();
    bool generateTrajectory(trajectory_msgs::JointTrajectory& traj);
    void publishGoalMarker();
    void publishMarkers();
    bool isObjectBlocked(std::vector<int>& idxs);
    void planInClutter(std::vector<int> idxs, std::vector<ob::ScopedState<ob::SE3StateSpace>>& states);

    ros::NodeHandle nh_;
    ros::Publisher marker_pub_;

    Manipulator* manipulator_;

    double plan_time_;
    util::CollisionGeometry target_geom_;

    std::vector<std::string> static_objs_;
    std::vector<util::CollisionGeometry> collision_boxes_;
    std::vector<og::PathGeometric> solutions_;
    
    Eigen::Vector3d goal_pos_;
    Eigen::Quaterniond goal_orient_;

	ob::StateSpacePtr space_;
	ob::SpaceInformationPtr si_;
	ob::ProblemDefinitionPtr pdef_;
    ob::PlannerPtr planner_;
    std::shared_ptr<StateChecker> state_checker_;
};