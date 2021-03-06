#pragma once

#include <fstream>
#include <ros/package.h>
#include <eigen3/Eigen/Geometry>
#include <boost/filesystem.hpp>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <gazebo_geometries_plugin/geometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <planner/start_plan.h>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/goals/GoalState.h>

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
    enum ActionType
    {
        NONE = -1,
        PUSH_GRASP,
        PUSH,
        GRASP
    };

    struct PlannerResult
    {
        bool path_found = false;
        bool partial_solution = false;
        bool grasp_success = false;
        int num_actions = 0;
        double plan_time = 0.0;
        double execution_time = 0.0;

        void reset()
        {
            path_found = partial_solution = grasp_success = false;
            num_actions = plan_time = execution_time = 0.0;
        }
    };

public:
    Planner(ros::NodeHandle* nh);
    ~Planner();

    bool plan();
    void setStart(const Eigen::Vector3d& start, const Eigen::Quaterniond& orientation);
    void setGoal(const Eigen::Vector3d& goal, const Eigen::Quaterniond& orientation);
    void setTargetGeometry(util::CollisionGeometry geom);
    void savePath();
    void clearMarkers();

private:
    void init();
    void initROS();
    bool generateTrajectory(trajectory_msgs::JointTrajectory& traj);
    void publishGoalMarker();
    void publishGridMap();
    void publishMarkers();
    void modelStatesCallback(gazebo_msgs::ModelStatesConstPtr msg);
    void update();
    bool verifyAndCorrectGraspPose(ob::ScopedState<ob::SE3StateSpace>& state);
    bool isObjectBlocked(std::vector<util::CollisionGeometry>& objs);
    Planner::ActionType planInClutter(const std::vector<util::CollisionGeometry>& objs,
        std::vector<ob::ScopedState<ob::SE3StateSpace>>& states);
    bool getPushAction(std::vector<ob::ScopedState<ob::SE3StateSpace>>& states, const std::vector<util::CollisionGeometry>& objs,
        const util::CollisionGeometry& geom);
    bool getGraspAction(std::vector<ob::ScopedState<ob::SE3StateSpace>>& states, const util::CollisionGeometry& geom);
    bool getPushGraspAction(const util::CollisionGeometry& geom, ob::ScopedState<ob::SE3StateSpace>& state);
    void executeAction(Planner::ActionType action);
    void resetArm();
    bool startPlanSrvCallback(planner::start_plan::Request& req, planner::start_plan::Response& res);
    bool resetArmSrvCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

    ros::NodeHandle nh_;
    ros::Publisher marker_pub_;
    ros::Publisher grid_pub_;
    ros::Subscriber models_sub_;
    ros::ServiceServer start_plan_srv_;
    ros::ServiceServer reset_arm_srv_;
    ros::ServiceClient collisions_client_;

    bool save_path_;
    int global_timeout_;
    int timeout_;
    int path_states_;
    std::string planner_name_;
    bool propagate_push_;
    bool use_grasping_;
    bool fix_grasp_pose_;

    Manipulator manipulator_;
    Controller controller_;

    PlannerResult result_;
    util::CollisionGeometry target_geom_;
    util::CollisionGeometry surface_geom_;
    std::string surface_parent_name_;
    util::Grid2D surface_grid_;
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
