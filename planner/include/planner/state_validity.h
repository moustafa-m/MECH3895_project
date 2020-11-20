#pragma once

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/MotionValidator.h>

//TODO Modify for Gazebo env

namespace ob = ompl::base;
namespace og = ompl::geometric;

class StateChecker : public ob::StateValidityChecker
{
public:
    StateChecker(const ob::SpaceInformationPtr &si)
        : ob::StateValidityChecker(si)
    {

    }

    bool isValid(const ob::State* state) const
    {
        return this->clearance(state) > 0.2;
    }

    double clearance(const ob::State* state) const
    {
        const ob::SE3StateSpace::StateType* state3D = state->as<ob::SE3StateSpace::StateType>();

        double x = state3D->getX(); double y = state3D->getY(); double z = state3D->getZ();

        return sqrt((x-1.5)*(x-1.5) + (y-0.68)*(y-0.68) + (z-0.03)*(z-0.03));
    }
    
private:

};