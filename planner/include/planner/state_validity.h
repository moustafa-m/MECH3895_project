#pragma once

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/MotionValidator.h>

#include <fcl/config.h>
#include <fcl/collision.h>
#include <fcl/broadphase/broadphase.h>
#include <fcl/math/transform.h>

#include "util.h"
#include "manipulator.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

class StateChecker : public ob::StateValidityChecker
{
public:
    StateChecker(const ob::SpaceInformationPtr &si, Manipulator* manip, const std::vector<util::CollisionGeometry>* collision_boxes)
        : ob::StateValidityChecker(si), manipulator_(manip), collision_boxes_(collision_boxes)
    {
        effector_translation_ = Eigen::Vector3d(0, 0.0, -0.12);
        finger_1_translation_ = Eigen::Vector3d(0.05, 0.0065, -0.032);
        finger_2_translation_ = Eigen::Vector3d(-0.045, 0.025, -0.032);
        finger_3_translation_ = Eigen::Vector3d(-0.045, -0.025, -0.032);
        finger_1_rotation_ = Eigen::Quaterniond(-0.571358774537, -0.644936902981, 0.287715328713, -0.418121312009);
        finger_2_rotation_ = Eigen::Quaterniond(0.295890700199, -0.408942259885, -0.577398820031, -0.641736335454);
        finger_3_rotation_ = Eigen::Quaterniond(0.408269313405, -0.295694272898, -0.643000046283, -0.57656916774);
    }

    bool isValid(const ob::State* state) const
    {
        if (!this->checkInReach(state)) { return false; }
        if (!this->checkCollision(state)) { return false; }
        return true;
    }

    bool checkInReach(const ob::State* state) const
    {
        const ob::SE3StateSpace::StateType* state3D = state->as<ob::SE3StateSpace::StateType>();
        return std::sqrt((state3D->getX()*state3D->getX())+(state3D->getY()*state3D->getY())+(state3D->getZ()*state3D->getZ())) <= 0.95;
    }

    bool checkCollision(const ob::State* state) const
    {
        const ob::SE3StateSpace::StateType* state3D = state->as<ob::SE3StateSpace::StateType>();
        double x = state3D->getX(); double y = state3D->getY(); double z = state3D->getZ();

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

        Eigen::Quaterniond rotation(state3D->rotation().w,
                            state3D->rotation().x,
                            state3D->rotation().y,
                            state3D->rotation().z);
        
        //TODO: this bit is very messy, need to clean it up somehow

        util::CollisionGeometry effector = collision_boxes_->at(idx);
        std::shared_ptr<fcl::CollisionGeometry> effectorbox;
        effectorbox = std::shared_ptr<fcl::CollisionGeometry>(new fcl::Box(effector.dimension.x, effector.dimension.y, effector.dimension.z));
        
        fcl::CollisionObject effectorobj(effectorbox);
        Eigen::Vector3d effector_translation = rotation.toRotationMatrix() * effector_translation_;
        fcl::Vec3f effector_xyz(x+effector_translation[0], y+effector_translation[1], z+effector_translation[2]); // set coordinates to be at centre of last link
        
        Eigen::Quaterniond q = rotation * Eigen::AngleAxisd(-M_PI, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitZ());
        fcl::Quaternion3f effector_quat(q.w(), q.x(), q.y(), q.z());
        
        effectorobj.setTransform(effector_quat, effector_xyz);

        // ----> fingers
        // ---> finger 1
        util::CollisionGeometry kinova_finger1 = collision_boxes_->at(collision_boxes_->size()-3);
        std::shared_ptr<fcl::CollisionGeometry> finger1box;
        finger1box = std::shared_ptr<fcl::CollisionGeometry>(new fcl::Box(kinova_finger1.dimension.x, kinova_finger1.dimension.y, kinova_finger1.dimension.z));
        
        fcl::CollisionObject finger1obj(finger1box);
        Eigen::Vector3d finger_1_translation = rotation.toRotationMatrix() * finger_1_translation_;
        fcl::Vec3f thumb_xyz(x+finger_1_translation[0], y+finger_1_translation[1], z+finger_1_translation[2]); // set coordinates to be centre of thum
        
        q = rotation * finger_1_rotation_;
        fcl::Quaternion3f finger1_quat(q.w(), q.x(), q.y(), q.z());
        
        finger1obj.setTransform(finger1_quat, thumb_xyz);
        // <--- finger 1

        // finger 2
        util::CollisionGeometry kinova_finger2 = collision_boxes_->at(collision_boxes_->size()-1);
        std::shared_ptr<fcl::CollisionGeometry> fingerbox2;
        fingerbox2 = std::shared_ptr<fcl::CollisionGeometry>(new fcl::Box(kinova_finger2.dimension.x, kinova_finger2.dimension.y, kinova_finger2.dimension.z));
        
        fcl::CollisionObject finger2obj(fingerbox2);
        Eigen::Vector3d finger_2_translation = rotation.toRotationMatrix() * finger_2_translation_;
        fcl::Vec3f finger_xyz(x+finger_2_translation[0], y+finger_2_translation[1], z+finger_2_translation[2]); // set coordinates to be centre of finger
        
        q = rotation * finger_2_rotation_;
        fcl::Quaternion3f finger2_quat(q.w(), q.x(), q.y(), q.z());
        
        finger2obj.setTransform(finger2_quat, finger_xyz);

        // finger 3
        util::CollisionGeometry kinova_finger3 = collision_boxes_->at(collision_boxes_->size()-2);
        std::shared_ptr<fcl::CollisionGeometry> fingerbox3;
        fingerbox3 = std::shared_ptr<fcl::CollisionGeometry>(new fcl::Box(kinova_finger3.dimension.x, kinova_finger3.dimension.y, kinova_finger3.dimension.z));
        
        fcl::CollisionObject finger3obj(fingerbox3);
        Eigen::Vector3d finger_3_translation = rotation.toRotationMatrix()*finger_3_translation_;
        fcl::Vec3f finger_xyz2(x+finger_3_translation[0], y+finger_3_translation[1], z+finger_3_translation[2]); // set coordinates to be centre of finger
        
        q = rotation * finger_3_rotation_;
        fcl::Quaternion3f finger3_quat(q.w(), q.x(), q.y(), q.z());
        
        finger3obj.setTransform(finger3_quat, finger_xyz2);
        // <---- fingers

        for (size_t i = 0; i < collision_boxes_->size(); i++)
        {
            // not checking for self collisions or collisions of other joints with environment
            if (collision_boxes_->at(i).name.find(manipulator_->getName()) != std::string::npos) { continue; }

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
            fcl::CollisionResult result, finger1_result, finger2_result, finger3_result;
            fcl::collide(&effectorobj, &obj, request, result);
            fcl::collide(&finger1obj, &obj, request, finger1_result);
            fcl::collide(&finger2obj, &obj, request, finger2_result);
            fcl::collide(&finger3obj, &obj, request, finger3_result);

            if (result.isCollision() || finger1_result.isCollision() || finger2_result.isCollision() || finger3_result.isCollision())
            {
                #ifdef DEBUG
                fcl::Vec3f translation; std::string name; geometry_msgs::Vector3 dim; fcl::Contact contact; fcl::Quaternion3f rot;

                std::cout << MAGENTA << "-----\n[DEBUG]\nCollision with: " << collision_boxes_->at(i).name << obj.getTranslation() << "\n"
                        << "Rotation: " << obj.getQuatRotation() << std::endl;
                std::cout << "Object dimensions: {" << collision_boxes_->at(i).dimension.x << " " << collision_boxes_->at(i).dimension.y << " " 
                        << collision_boxes_->at(i).dimension.z << "}" << std::endl;
                std::cout << "Detected collision in state: {" << x << ", " << y << ", " << z << "}" << std::endl;
                
                if (result.isCollision()) { translation = effectorobj.getTranslation(); name = "effector"; dim = effector.dimension; rot = effector_quat; }
                else if (finger1_result.isCollision()) { translation = finger1obj.getTranslation(); name = "finger1"; dim = kinova_finger1.dimension; rot = finger1_quat; }
                else if (finger2_result.isCollision()) { translation = finger2obj.getTranslation(); name = "finger2"; dim = kinova_finger2.dimension; rot = finger2_quat; }
                else { translation = finger3obj.getTranslation(); name = "finger3"; dim = kinova_finger3.dimension; rot = finger3_quat; }

                std::cout << "link name: " << name << "\nlink position: " << translation << "\nRotation: " << rot << "\nlink dimensions: {"
                        << dim.x << ", " << dim.y << ", " << dim.z << "}\n-----" << std::endl;
                #endif
                return false;
            }
        }
    
        return true;
    }
    
private:
    Manipulator* manipulator_;
    const std::vector<util::CollisionGeometry>* collision_boxes_;

    // these transformations are the same for the 3 finger Jaco arms
    Eigen::Vector3d effector_translation_;
    Eigen::Vector3d finger_1_translation_;
    Eigen::Vector3d finger_2_translation_;
    Eigen::Vector3d finger_3_translation_;
    Eigen::Quaterniond finger_1_rotation_;
    Eigen::Quaterniond finger_2_rotation_;
    Eigen::Quaterniond finger_3_rotation_;

};