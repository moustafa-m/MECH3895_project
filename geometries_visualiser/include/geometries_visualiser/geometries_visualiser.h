#pragma once

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <gazebo_geometries_plugin/geometry.h>
#include <gazebo_msgs/ModelStates.h>

#define GREEN "\033[0;32m"
#define NC "\033[0m"

class GeometriesVisualiser
{
public:

    GeometriesVisualiser(ros::NodeHandle* nh);
    ~GeometriesVisualiser();

    void run();
    
private:
    void init();
    void statesCallback(gazebo_msgs::ModelStatesConstPtr msg);
    void visGeometries();

    ros::NodeHandle nh_;
    ros::Publisher geometries_marker_pub_;
    ros::Subscriber states_sub_;
    ros::ServiceClient geometries_client_;

    std::string robot_name_;
    std::string geometries_ns_;
    std::string base_frame_id_;

    gazebo_msgs::ModelStates states_;
    
    std::vector<std::string> static_objs_;
};