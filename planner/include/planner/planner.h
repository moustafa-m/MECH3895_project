#pragma once

#include <fstream>
#include <ros/package.h>
#include <eigen3/Eigen/Geometry>
#include <boost/filesystem.hpp>
#include <visualization_msgs/Marker.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <gazebo_geometries_plugin/geometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <planner/start_plan.h>

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
#include "timer.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

class Planner
{
public:
    // TODO: use this in implementation
    enum ActionType
    {
        NONE = -1,
        PUSH_GRASP,
        PUSH,
        GRASP
    };

    typedef struct PlannerResult
    {
        bool path_found = false;
        bool path_valid = false;
        bool grasp_success = false;
        double plan_time = 0.0;
        double execution_time = 0.0;

        void reset()
        {
            path_found = path_valid = grasp_success = false;
            plan_time = execution_time = 0.0;
        }
    } PlannerResult;

public:
    Planner(ros::NodeHandle* nh);
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
    Planner::ActionType planInClutter(std::vector<int> idxs, std::vector<ob::ScopedState<ob::SE3StateSpace>>& states);
    bool getPushAction(std::vector<ob::ScopedState<ob::SE3StateSpace>>& states, std::vector<util::CollisionGeometry>& objs,
        const util::CollisionGeometry& geom);
    bool getPushGraspAction(const util::CollisionGeometry& geom, ob::ScopedState<ob::SE3StateSpace>& state);
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

    PlannerResult result_;
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