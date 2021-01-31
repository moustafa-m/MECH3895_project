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
#include "controller.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

class Planner
{
public:
    Planner(ros::NodeHandle* nh, Manipulator* manip);
    ~Planner();

    bool plan();
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
    void modelStatesCallback(const gazebo_msgs::ModelStatesConstPtr msg);
    void update();
    bool isObjectBlocked(std::vector<int>& idxs);
    void planInClutter(std::vector<int> idxs, std::vector<ob::ScopedState<ob::SE3StateSpace>>& states);
    bool getPushAction(std::vector<ob::ScopedState<ob::SE3StateSpace>>& states, std::vector<util::CollisionGeometry>& objs,
        const util::CollisionGeometry& geom);
    bool startPlanSrvCallback(planner::start_plan::Request& req, planner::start_plan::Response& res);

    ros::NodeHandle nh_;
    ros::Publisher marker_pub_;
    ros::Subscriber models_sub_;
    ros::ServiceServer start_plan_srv_;
    ros::ServiceClient collisions_client_;

    bool save_path_;
    bool display_path_;
    int timeout_;
    int path_states_;
    std::string planner_name_;

    Manipulator manipulator_;
    Controller controller_;

    double plan_time_;
    util::CollisionGeometry target_geom_;
    gazebo_msgs::ModelStatesConstPtr models_;

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