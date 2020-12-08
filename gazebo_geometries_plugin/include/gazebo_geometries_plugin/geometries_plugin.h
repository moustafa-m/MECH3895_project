#pragma once

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_geometries_plugin/geometry.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/common/Plugin.hh>
#include <ignition/math/Vector3.hh>

#define GREEN "\033[0;32m"

namespace gazebo
{
    class GeometriesPlugin : public WorldPlugin
    {
    public:
        GeometriesPlugin();
        ~GeometriesPlugin();

        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    private:
        void getBBox(gazebo_geometries_plugin::geometry::Response &res, gazebo::physics::CollisionPtr geom);
        bool getGeometrySrv(gazebo_geometries_plugin::geometry::Request &req, gazebo_geometries_plugin::geometry::Response &res);

        ros::NodeHandle nh_;
        ros::ServiceServer get_geometry_srv;

        physics::WorldPtr world_;
    };

GZ_REGISTER_WORLD_PLUGIN(GeometriesPlugin)
}