#pragma once

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/MotionValidator.h>

#include <fcl/config.h>
#include <fcl/collision.h>
#include <fcl/broadphase/broadphase.h>
#include <fcl/math/transform.h>

#include "util.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

class StateChecker : public ob::StateValidityChecker
{
public:
    StateChecker(const ob::SpaceInformationPtr &si, const std::string* manip_name, const std::vector<util::CollisionGeometry>* collision_boxes)
        : ob::StateValidityChecker(si), manipulator_name_(manip_name), collision_boxes_(collision_boxes)
    {

    }

    bool isValid(const ob::State* state) const
    {
        return this->checkValidity(state);
    }

    bool checkValidity(const ob::State* state) const
    {
        const ob::SE3StateSpace::StateType* state3D = state->as<ob::SE3StateSpace::StateType>();

        double x = state3D->getX(); double y = state3D->getY(); double z = state3D->getZ();
        if (std::sqrt((x*x)+(y*y)+(z*z)) >= 0.95)
        {
            #ifdef DEBUG
            std::cout << MAGENTA << "-----\n[DEBUG]\n" << "State {" << x << ", " << y << ", " << z
                    << "} out of reach: " << std::sqrt((x*x)+(y*y)+(z*z)) << "m\n-----" << std::endl;
            #endif
            return false;
        }

        int idx = 0;
        for (size_t i = 0; i < collision_boxes_->size(); i++)
        {
            if (collision_boxes_->at(i).name.find("_link_7") != std::string::npos)
            {
                idx = i;
                break;
            }
            else if (collision_boxes_->at(i).name.find("_link_6") != std::string::npos)
            {
                idx = i;
            }
        }

        auto sign = [](double val) { return val < 0 ? -1 : 1; };
        
        // NOTE: the last link's bounding box is slightly enlarged to enforce some clearance.
        // The orientation of the end effector and fingers is assumed to be the same as the start orientation,
        // this makes it easier to use the AABB obtained from Gazebo
        double x_offset = 0.05, y_offset = 0.02;
        util::CollisionGeometry effector = collision_boxes_->at(idx);
        effector.dimension.x += x_offset; effector.dimension.y += y_offset;
        std::shared_ptr<fcl::CollisionGeometry> effectorbox;
        effectorbox = std::shared_ptr<fcl::CollisionGeometry>(new fcl::Box(effector.dimension.x, effector.dimension.y, effector.dimension.z));
        fcl::CollisionObject effectorobj(effectorbox);
        fcl::Vec3f effector_xyz(x-(sign(x)*(0.12+(effector.dimension.x/2))), y, z); // change center of collision box to be at centre of last link
        fcl::Quaternion3f effector_quat(1, 0, 0, 0); // this is an AABB, so no rotation
        effectorobj.setTransform(effector_quat, effector_xyz);

        // Thumb
        util::CollisionGeometry kinova_thumb = collision_boxes_->at(collision_boxes_->size()-3);
        std::shared_ptr<fcl::CollisionGeometry> thumbbox;
        thumbbox = std::shared_ptr<fcl::CollisionGeometry>(new fcl::Box(kinova_thumb.dimension.x+0.04, kinova_thumb.dimension.y, kinova_thumb.dimension.z));
        fcl::CollisionObject thumbobj(thumbbox);
        fcl::Vec3f thumb_xyz(x-(sign(x)*0.05), y+0.08, z); // change center of thumb
        // fcl::Quaternion3f thumb_quat(0.1542514, 0, 0, 0.9880316);
        fcl::Quaternion3f thumb_quat(1, 0, 0, 0);
        thumbobj.setTransform(thumb_quat, thumb_xyz);

        // fingers NOTE: the collision geometry for the two fingers is taken to be one box for simplification hence
        // z dimension is multiplied by 2 and has 2cm added
        util::CollisionGeometry kinova_finger = collision_boxes_->at(collision_boxes_->size()-2);
        std::shared_ptr<fcl::CollisionGeometry> fingerbox;
        fingerbox = std::shared_ptr<fcl::CollisionGeometry>(new fcl::Box(kinova_finger.dimension.x+0.04, kinova_finger.dimension.y, kinova_finger.dimension.z*2+0.02));
        fcl::CollisionObject fingerobj(fingerbox);
        fcl::Vec3f finger_xyz(x-(sign(x)*0.05), y-0.08, z); // change center of fingers
        // fcl::Quaternion3f finger_quat(0.1542514, 0, 0, -0.9880316);
        fcl::Quaternion3f finger_quat(1, 0, 0, 0);
        fingerobj.setTransform(finger_quat, finger_xyz);

        for (size_t i = 0; i < collision_boxes_->size(); i++)
        {
            // not checking for self collisions or collisions of other joints with environment
            if (collision_boxes_->at(i).name.find(*manipulator_name_) != std::string::npos) { continue; }

            std::shared_ptr<fcl::CollisionGeometry> box;
            box = std::shared_ptr<fcl::CollisionGeometry>(new fcl::Box(collision_boxes_->at(i).dimension.x, collision_boxes_->at(i).dimension.y, collision_boxes_->at(i).dimension.z));

            fcl::CollisionObject obj(box);
            fcl::Vec3f xyz(collision_boxes_->at(i).pose.position.x, collision_boxes_->at(i).pose.position.y, collision_boxes_->at(i).pose.position.z);
            // fcl::Quaternion3f quat(1, 0, 0, 0);
            fcl::Quaternion3f quat(collision_boxes_->at(i).pose.orientation.w,
                            collision_boxes_->at(i).pose.orientation.x,
                            collision_boxes_->at(i).pose.orientation.y,
                            collision_boxes_->at(i).pose.orientation.z);
            obj.setTransform(quat, xyz);

            fcl::CollisionRequest request(1, false, 1, false);
            fcl::CollisionResult result, thumb_result, finger_result;
            fcl::collide(&effectorobj, &obj, request, result);
            fcl::collide(&thumbobj, &obj, request, thumb_result);
            fcl::collide(&fingerobj, &obj, request, finger_result);

            if (result.isCollision() || thumb_result.isCollision() || finger_result.isCollision())
            {
                #ifdef DEBUG
                fcl::Vec3f translation; std::string name; geometry_msgs::Vector3 dim; fcl::Contact contact; fcl::Quaternion3f rot;

                std::cout << MAGENTA << "-----\n[DEBUG]\nCollision with: " << collision_boxes_->at(i).name << obj.getTranslation() << "\n"
                        << "Rotation: " << obj.getQuatRotation() << std::endl;
                std::cout << "Object dimensions: {" << collision_boxes_->at(i).dimension.x << " " << collision_boxes_->at(i).dimension.y << " " 
                        << collision_boxes_->at(i).dimension.z << "}" << std::endl;
                std::cout << "Detected collision in state: {" << x << ", " << y << ", " << z << "}" << std::endl;
                
                if (result.isCollision()) { translation = effectorobj.getTranslation(); name = "effector"; dim = effector.dimension; rot = effector_quat; }
                else if (thumb_result.isCollision()) { translation = thumbobj.getTranslation(); name = "thumb"; dim = kinova_thumb.dimension; rot = thumb_quat; }
                else { translation = fingerobj.getTranslation(); name = "finger"; dim = kinova_finger.dimension; rot = finger_quat; }

                std::cout << "link name: " << name << "\nlink position: " << translation << "\nRotation: " << rot << "\nlink dimensions: {"
                        << dim.x << ", " << dim.y << ", " << dim.z << "}\n-----" << std::endl;
                #endif
                return false;
            }
        }
    
        return true;
    }
    
private:
    const std::vector<util::CollisionGeometry>* collision_boxes_;
    const std::string* manipulator_name_;

};