#pragma once

#include <fstream>
#include <ros/package.h>
#include <eigen3/Eigen/Geometry>
#include <boost/filesystem.hpp>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/spaces/SE3StateSpace.h>

#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/SimpleSetup.h>

#include "state_validity.h"
#include "util.h"
#include "defines.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

class Planner
{
public:
    Planner();
    ~Planner();

    og::PathGeometric plan();
    void setStart(const Eigen::Vector3d& start, const Eigen::Quaterniond& orientation);
    void setGoal(const Eigen::Vector3d& goal, const Eigen::Quaterniond& orientation, const std::string& obj_name);
    void setCollisionGeometries(const std::vector<util::CollisionGeometry>& collision_boxes);
    void setManipulatorName(const std::string& name);
    void savePath();

private:
    void init();

    std::string name_;
    double plan_time_;

    std::vector<util::CollisionGeometry> collision_boxes_;
    std::string target_name_;
    std::string manipulator_name_;

	ob::StateSpacePtr space_;
	ob::SpaceInformationPtr si_;
	ob::ProblemDefinitionPtr pdef_;
};