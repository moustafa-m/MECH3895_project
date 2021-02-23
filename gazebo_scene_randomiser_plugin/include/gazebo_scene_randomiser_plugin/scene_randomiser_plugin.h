#pragma once

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <chrono>
#include <gazebo_scene_randomiser_plugin/randomise.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/common/Plugin.hh>
#include <ignition/math/Vector3.hh>

#define GREEN "\033[0;32m"

namespace gazebo
{
    class SceneRandomiser : public WorldPlugin
    {
    public:
        SceneRandomiser();
        ~SceneRandomiser();

        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    private:
        double getModelArea(physics::ModelPtr model);
        bool checkCollision(ignition::math::v4::Pose3d& pose, physics::ModelPtr model, physics::ModelPtr parent);
        ignition::math::v4::Pose3d getModelPose(const ignition::math::v4::Pose3d& centre, physics::ShapePtr shape);
        bool randomiseSrvCallback(gazebo_scene_randomiser_plugin::randomiseRequest& req, gazebo_scene_randomiser_plugin::randomiseResponse& res);

        std::unique_ptr<ros::NodeHandle> nh_;
        ros::ServiceServer randomise_srv_;

        std::string robot_name_;

        physics::WorldPtr world_;
    };

GZ_REGISTER_WORLD_PLUGIN(SceneRandomiser)
}
