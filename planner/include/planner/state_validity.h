#pragma once

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/MotionValidator.h>

#include <fcl/config.h>
#include <fcl/collision.h>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/math/transform.h>

#include "util.h"
#include "manipulator.h"

namespace ob = ompl::base;

class StateChecker : public ob::StateValidityChecker
{
public:
    StateChecker(const ob::SpaceInformationPtr &si, Manipulator* manip, const std::vector<util::CollisionGeometry>* collision_boxes);

    void setTargetCollision(bool option);
    void setNonStaticCollisions(bool option);
    void setTargetName(std::string name);
    void setIKCheck(bool option);
    bool isValid(const ob::State* state) const;
    bool checkInReach(const ob::State* state) const;
    bool checkIK(const ob::State* state) const;
    bool checkCollision(const ob::State* state) const;

private:
    Manipulator* manipulator_;
    const std::vector<util::CollisionGeometry>* collision_boxes_;
    std::vector<std::string> static_objs_;
    std::string target_name_;
    bool check_IK_;
    
    bool target_collision_;
    bool non_static_collisions_;

    // these transformations are the same for the 3 finger Jaco arms
    Eigen::Vector3d effector_translation_;
    Eigen::Vector3d finger_1_translation_;
    Eigen::Vector3d finger_2_translation_;
    Eigen::Vector3d finger_3_translation_;
    Eigen::Quaterniond finger_1_rotation_;
    Eigen::Quaterniond finger_2_rotation_;
    Eigen::Quaterniond finger_3_rotation_;

};
