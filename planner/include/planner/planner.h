#pragma once

#include <fstream>
#include <ros/package.h>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/spaces/SE3StateSpace.h>

#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/SimpleSetup.h>
#include "state_validity.h"
#include "defines.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

class Planner
{
public:
    Planner();
    ~Planner();

    og::PathGeometric plan();
    void setStart(Eigen::Vector3d& start);
    void setGoal(Eigen::Vector3d& goal);
    void savePath(const og::PathGeometric& path);

private:
    void init();

	ob::StateSpacePtr space_;
	ob::SpaceInformationPtr si_;
	ob::ProblemDefinitionPtr pdef_;
};