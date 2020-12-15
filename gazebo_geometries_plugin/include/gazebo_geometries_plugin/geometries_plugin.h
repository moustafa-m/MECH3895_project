#pragma once

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <visualization_msgs/Marker.h>
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
        void queueThread();
        void initKinovaDimensions();
        void publishMarkers();
        void getBBox(gazebo_geometries_plugin::geometry::Response &res, gazebo::physics::CollisionPtr geom);
        bool getGeometrySrv(gazebo_geometries_plugin::geometry::Request &req, gazebo_geometries_plugin::geometry::Response &res);

        bool pub_arm_geom_;
        
        std::unique_ptr<ros::NodeHandle> nh_;
        ros::CallbackQueue queue_;
        std::thread queue_thread_;

        ros::Publisher marker_pub_;
        ros::ServiceServer get_geometry_srv;

        physics::WorldPtr world_;

        std::string geometries_ns_;
        std::string base_frame_id_;

        std::vector<std::string> static_objs_;
        std::string robot_name_;
        std::vector<geometry_msgs::Vector3> kinova_dimensions_;
    };

GZ_REGISTER_WORLD_PLUGIN(GeometriesPlugin)
}