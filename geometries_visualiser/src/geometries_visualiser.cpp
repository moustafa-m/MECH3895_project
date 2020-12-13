#include "geometries_visualiser/geometries_visualiser.h"

GeometriesVisualiser::GeometriesVisualiser(ros::NodeHandle* nh)
    : nh_(*nh)
{
    this->init();
    ROS_INFO("%s[GeomtriesVisualiser]: Initialised!", GREEN);
}

GeometriesVisualiser::~GeometriesVisualiser()
{

}

void GeometriesVisualiser::run()
{
    this->visGeometries();
}

void GeometriesVisualiser::init()
{
    ros::param::param<std::string>("/robot_name", robot_name_, "j2s7s300");
    base_frame_id_ = robot_name_ + "_link_base";

    geometries_ns_ = "geometries";

    states_sub_ = nh_.subscribe("/gazebo/model_states", 10, &GeometriesVisualiser::statesCallback, this);

    geometries_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/geometries_markers", 10);

    geometries_client_ = nh_.serviceClient<gazebo_geometries_plugin::geometry>("/gazebo/get_geometry");
}

void GeometriesVisualiser::statesCallback(gazebo_msgs::ModelStatesConstPtr msg)
{
    states_ = *msg;
}

void GeometriesVisualiser::visGeometries()
{
    if (states_.name.empty()) return;
    
    visualization_msgs::Marker marker;
    int id = 0;

    for (size_t i = 0; i < states_.name.size(); i++)
    {
        if (states_.name[i].find(robot_name_) != std::string::npos || states_.name[i].find("ground_plane") != std::string::npos) continue;

        gazebo_geometries_plugin::geometry srv;
        srv.request.model_name = states_.name[i];
        if (geometries_client_.call(srv))
        {
            for (size_t j = 0; j < srv.response.name.size(); j++)
            {
                marker.id = id;
                marker.action = marker.ADD;
                marker.type = marker.CUBE;
                
                marker.color.r = 0.8;
                marker.color.g = 0.2;
                marker.color.b = 0.8;
                marker.color.a = 1.0;

                marker.pose = srv.response.pose[j];
                marker.scale = srv.response.dimensions[j];

                marker.header.frame_id = base_frame_id_;
                marker.header.stamp = ros::Time::now();
                marker.header.seq += 1;

                marker.ns = geometries_ns_;

                geometries_marker_pub_.publish(marker);

                id++;
            }
        }
    }
}