#include "gazebo_geometries_plugin/geometries_plugin.h"

using namespace gazebo;

GeometriesPlugin::GeometriesPlugin()
    : WorldPlugin()
{
    get_geometry_srv = nh_.advertiseService("/gazebo/get_geometry", &GeometriesPlugin::getGeometrySrv, this);
}

GeometriesPlugin::~GeometriesPlugin()
{

}

void GeometriesPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
    if (!ros::isInitialized())
    {
        ROS_FATAL("Gazebo ROS node not initialised!");
        return;
    }

    world_ = _world;
    ROS_INFO("%sGeometriesPlugin: Plugin loaded!", GREEN);
}

bool GeometriesPlugin::getGeometrySrv(gazebo_geometries_plugin::geometry::Request &req, gazebo_geometries_plugin::geometry::Response &res)
{
    physics::ModelPtr model = world_->ModelByName(req.model_name);

    if (!model)
    {
        ROS_ERROR("GeometriesPlugin: Unable to find [%s]! Model does not exist", req.model_name.c_str());

        res.message = "Error, model does not exist!";
        geometry_msgs::Vector3 size;
        size.x = size.y = size.z = NAN;
        res.min_bounds = res.max_bounds = size;
        return false;
    }

    ROS_INFO("%sGeometriesPlugin: Found [%s]!", GREEN, req.model_name.c_str());

    res.message = "Model found!";

    ignition::math::Box box = model->BoundingBox();

    geometry_msgs::Vector3 min, max, centre;

    min.x = box.Min().X(); min.y = box.Min().Y(); min.z = box.Min().Z();
    max.x = box.Max().X(); max.y = box.Max().Y(); max.z = box.Max().Z();
    centre.x = box.Center().X(); centre.y = box.Center().Y(); centre.z = box.Center().Z();

    res.min_bounds = min; res.max_bounds = max; res.centre = centre;

    return true;
}