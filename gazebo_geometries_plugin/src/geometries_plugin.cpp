#include "gazebo_geometries_plugin/geometries_plugin.h"

using namespace gazebo;

GeometriesPlugin::GeometriesPlugin()
    : WorldPlugin()
{

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

    nh_.reset(new ros::NodeHandle("geometries_plugin"));
    nh_->setCallbackQueue(&queue_);

    ros::param::param<std::vector<std::string>>("/gazebo/static_objects", static_objs_, {"INVALID"});
    ros::param::param<std::string>("/robot_name", robot_name_, "j2s7s300");

    this->initKinovaDimensions();

    base_frame_id_ = robot_name_ + "_link_base";
    geometries_ns_ = "geometries";

    marker_pub_ = nh_->advertise<visualization_msgs::Marker>("/geometries_markers", 10);
    get_geometry_srv = nh_->advertiseService("/gazebo/get_geometry", &GeometriesPlugin::getGeometrySrv, this);

    queue_thread_ = std::thread(std::bind(&GeometriesPlugin::queueThread, this));

    world_ = _world;
    ROS_INFO("%sGeometriesPlugin: Plugin loaded!", GREEN);
}

void GeometriesPlugin::queueThread()
{
    ros::Rate rate(5);
    while (nh_->ok())
    {
        queue_.callAvailable(ros::WallDuration(0.05));
        this->publishMarkers();
        rate.sleep();
    }
}

void GeometriesPlugin::initKinovaDimensions()
{
    using namespace geometry_msgs;

    // fingers
    Vector3 link_finger_1, link_finger_2, link_finger_3;
    
    // links (names match the stl files in kinova_description package)
    Vector3 base, shoulder, arm, arm_half_1, arm_half_2, forearm, wrist, wrist_spherical_1, wrist_spherical_2, hand_3finger;

    // values are obtained by measuring mesh files in kinova_description package
    link_finger_1.x = 0.13, link_finger_1.y = 0.025, link_finger_1.z = 0.025;
    link_finger_2 = link_finger_3 = link_finger_1;
    
    base.x = 0.083, base.y = 0.083, base.z = 0.16;
    shoulder.x = 0.083, shoulder.y = 0.083, shoulder.z = 0.16;
    arm.x = 0.083, arm.y = 0.5, arm.z = 0.42;
    arm_half_1.x = 0.083, arm_half_1.y = 0.245, arm_half_1.z = 0.0855;
    arm_half_2.x = 0.083, arm_half_2.y = 0.0855, arm_half_2.z = 0.245;
    forearm.x = 0.082, forearm.y = 0.248, forearm.z = 0.063;
    wrist.x = 0.065, wrist.y = 0.085, wrist.z = 0.09;                                       // curved wrist
    wrist_spherical_1.x = 0.063, wrist_spherical_1.y = 0.063, wrist_spherical_1.z = 0.15;
    wrist_spherical_2.x = 0.063, wrist_spherical_2.y = 0.15, wrist_spherical_2.z = 0.09;
    hand_3finger.x = 0.085, hand_3finger.y = 0.10, hand_3finger.z = 0.12;

    if (robot_name_.compare("j2s7s300") == 0)
    {
        kinova_dimensions_ = {base, shoulder, arm_half_1, arm_half_2, forearm, wrist_spherical_1, wrist_spherical_2, hand_3finger,
            link_finger_1, link_finger_2, link_finger_3};
    }
    else if (robot_name_.compare("j2s6s300") == 0)
    {
        kinova_dimensions_ = {base, shoulder, arm, forearm, wrist_spherical_1, wrist_spherical_2, hand_3finger, link_finger_1,
            link_finger_2, link_finger_3};
    }
    else if (robot_name_.compare("j2n6s300") == 0)
    {
        kinova_dimensions_ = {base, shoulder, arm, forearm, wrist, wrist, hand_3finger, link_finger_1, link_finger_2, link_finger_3};
    }
}

void GeometriesPlugin::publishMarkers()
{
    visualization_msgs::Marker marker;
    int id = 0;
    bool is_static = false;

    std::vector<gazebo::physics::ModelPtr> models = world_->Models();
    if (models.empty()) return;

    for (size_t i = 0; i < models.size(); i++)
    {
        if (models[i]->GetName().find(robot_name_) != std::string::npos||
            models[i]->GetName().find("ground_plane") != std::string::npos)
            { continue; }

        is_static = std::any_of(static_objs_.begin(), static_objs_.end(), [this, i, models](const std::string& str)
            { return models[i]->GetName().find(str) != std::string::npos; });

        // for (size_t j = 0; j < static_objs_.size(); j++)
        // {
        //     is_static = states_.name[i].find(static_objs_[j]) != std::string::npos;

        //     if (is_static)
        //     {
        //         // std::cout << static_objs_[j] << " " << states_.name[i] << std::endl;
        //         break;
        //     }
        // }

        gazebo_geometries_plugin::geometry srv;
        srv.request.model_name = models[i]->GetName();
        if (this->getGeometrySrv(srv.request, srv.response))
        {
            for (size_t j = 0; j < srv.response.name.size(); j++)
            {
                if (!std::isfinite(srv.response.dimensions[j].x) || std::abs(srv.response.dimensions[j].x) <= 1e-5) continue;

                // std::cout << srv.response.dimensions[j] << std::endl;
                
                marker.id = id;
                marker.action = marker.ADD;
                marker.type = marker.CUBE;
                
                if (is_static) { marker.color.r = marker.color.a = 1.0; marker.color.g = marker.color.b = 0.0; }
                else { marker.color.g = marker.color.a = 1.0; marker.color.r = marker.color.b = 0.0; }

                marker.pose = srv.response.pose[j];
                marker.scale = srv.response.dimensions[j];

                marker.header.frame_id = base_frame_id_;
                marker.header.stamp = ros::Time::now();
                marker.header.seq += 1;
                marker.ns = geometries_ns_;

                marker_pub_.publish(marker);
                id++;
            }
        }
    }
}

void GeometriesPlugin::getBBox(gazebo_geometries_plugin::geometry::Response &res, gazebo::physics::CollisionPtr geom)
{
    ignition::math::Box collision_box;
    geometry_msgs::Vector3 min, max, dimensions;
    geometry_msgs::Pose pose;

    gazebo::physics::ShapePtr shape(geom->GetShape());
    collision_box = geom->CollisionBoundingBox();
    min.x = collision_box.Min().X(); min.y = collision_box.Min().Y(); min.z = collision_box.Min().Z();
    max.x = collision_box.Max().X(); max.y = collision_box.Max().Y(); max.z = collision_box.Max().Z();
    pose.position.x = collision_box.Center().X(); pose.position.y = collision_box.Center().Y(); pose.position.z = collision_box.Center().Z();
    
    if (res.name[0].find(robot_name_) != std::string::npos)
    {
        if (geom->GetName().find("finger_tip") != std::string::npos)
        {
            res.name.pop_back();
            return;
        }
        else if (geom->GetName().find("finger") != std::string::npos)
        {
            res.dimensions.push_back(kinova_dimensions_.back());
        }
        else
        {
            std::string substr = geom->GetName().substr(geom->GetName().find("link_"));
            int idx = 0;
            for (int i = 0; i < substr.length(); i++)
            {
                if (std::isdigit(substr[i]))
                {
                    idx = std::stoi(substr.substr(i, 1));
                    break;
                }
            }
            res.dimensions.push_back(kinova_dimensions_[idx]);
        }

        pose.orientation.x = geom->WorldPose().Rot().X(); pose.orientation.y = geom->WorldPose().Rot().Y(); pose.orientation.z = geom->WorldPose().Rot().Z();
        pose.orientation.w = geom->WorldPose().Rot().W();

        res.min_bounds.push_back(min);
        res.max_bounds.push_back(max);
        res.pose.push_back(pose);
        return;
    }
    
    // if the geometry has a box shape, an Object Oriented Bounding Box (OOBB) can be obtained directly, otherwise
    // an Axis Aligned Bounding Box (AABB) is obtained. The AABB is aligned with the world axes, whereas the
    // OOBB is aligned with object axes
    if (shape->HasType(gazebo::physics::Shape::BOX_SHAPE))
    {
        gazebo::physics::BoxShape *box = static_cast<gazebo::physics::BoxShape*>(shape.get());

        dimensions.x = box->Size().X(); dimensions.y = box->Size().Y(); dimensions.z = box->Size().Z();
        pose.orientation.x = geom->WorldPose().Rot().X(); pose.orientation.y = geom->WorldPose().Rot().Y(); pose.orientation.z = geom->WorldPose().Rot().Z();
        pose.orientation.w = geom->WorldPose().Rot().W();
    }
    else if (shape->HasType(gazebo::physics::Shape::CYLINDER_SHAPE))
    {
        // cylinders will be assumed to be box shaped to obtain OOBB

        gazebo::physics::CylinderShape *cylinder = static_cast<gazebo::physics::CylinderShape*>(shape.get());

        dimensions.x = cylinder->GetRadius()*2; dimensions.y = cylinder->GetRadius()*2;
        dimensions.z = cylinder->GetLength();

        pose.orientation.x = geom->WorldPose().Rot().X(); pose.orientation.y = geom->WorldPose().Rot().Y(); pose.orientation.z = geom->WorldPose().Rot().Z();
        pose.orientation.w = geom->WorldPose().Rot().W();
    }
    else
    {
        dimensions.x = collision_box.Size().X(); dimensions.y = collision_box.Size().Y(); dimensions.z = collision_box.Size().Z();
        pose.orientation.x = 0; pose.orientation.y = 0; pose.orientation.z = 0;
        pose.orientation.w = 1;
    }

    res.min_bounds.push_back(min);
    res.max_bounds.push_back(max);
    res.dimensions.push_back(dimensions);
    res.pose.push_back(pose);
}

bool GeometriesPlugin::getGeometrySrv(gazebo_geometries_plugin::geometry::Request &req, gazebo_geometries_plugin::geometry::Response &res)
{
    physics::ModelPtr model;
    bool find_link = false;

    // first check if user is asking for a link's geometry
    std::string::size_type pos = req.model_name.find("_link");
    if (pos != std::string::npos)
    {
        find_link = true;

        // ROS_INFO("%sGeometriesPlugin: Requests for link [%s] recieved!", GREEN, req.model_name.c_str());
        std::string name = req.model_name.substr(0, pos);
        // ROS_INFO("%sGeometriesPlugin: Looking for parent [%s]", GREEN, name.c_str());
        model = world_->ModelByName(name);
    }
    else
    {
        model = world_->ModelByName(req.model_name);
        // ROS_INFO("%sGeometriesPlugin: Looking for [%s]", GREEN, req.model_name.c_str());
    }
    
    if (!model)
    {
        ROS_ERROR("GeometriesPlugin: Unable to find [%s]! Model does not exist", req.model_name.c_str());

        res.message = "Error, model does not exist!";
        geometry_msgs::Vector3 size;
        // size.x = size.y = size.z = NAN;
        // res.min_bounds = res.max_bounds = size;
        return false;
    }

    // ROS_INFO("%sGeometriesPlugin: Found %d child links for [%s]", GREEN, model->GetChildCount(), req.model_name.c_str());
    
    // loop through parent model child links
    for (unsigned int i = 0 ; i < model->GetChildCount(); i ++)
    {
        gazebo::physics::LinkPtr body = boost::dynamic_pointer_cast<gazebo::physics::Link>(model->GetChild(i));

        if (body)
        {
            std::size_t npos = std::string::npos;
            // NOTE: kinova model has multiple child links, each with multiple collision geometry, default gazebo models have one
            // child link with multiple collision geometries
            std::string::size_type pos = req.model_name.find("link_");
            std::string link_name = req.model_name.substr(pos+5, req.model_name.length());
            if (req.model_name.find("j2s7s300") != npos || req.model_name.find("j2s6s300") != npos || req.model_name.find("j2n6s300") != npos)
            {
                link_name = req.model_name + "_collision";
            }
            
            for (unsigned int j = 0; j < body->GetChildCount() ; j++)
            {
                // check if user is asking for a specific link
                if (!find_link)
                {
                    gazebo::physics::CollisionPtr geom = boost::dynamic_pointer_cast<gazebo::physics::Collision>(body->GetChild(j));
                    res.name.push_back(req.model_name + "_" + body->GetChild(j)->GetName());
                    getBBox(res, geom);
                }
                else if (body->GetChild(j)->GetName().compare(link_name) == 0)
                {
                    // ROS_INFO("%sGeometriesPlugin: Found [%s]!", GREEN, link_name.c_str());
                    gazebo::physics::CollisionPtr geom = boost::dynamic_pointer_cast<gazebo::physics::Collision>(body->GetChild(j));
                    res.name.push_back(req.model_name);
                    getBBox(res, geom);
                    break;
                }
            }
        }
    }
    
    res.message = "GeomtriesPlugin: Model found!";
    // ROS_INFO("%sGeometriesPlugin: Obtained collision geometries for [%s]", GREEN, req.model_name.c_str());

    return true;
}