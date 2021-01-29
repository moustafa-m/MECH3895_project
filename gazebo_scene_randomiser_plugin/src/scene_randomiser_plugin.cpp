#include "gazebo_scene_randomiser_plugin/scene_randomiser_plugin.h"

using namespace gazebo;
namespace chrono = std::chrono;

SceneRandomiser::SceneRandomiser()
{

}

SceneRandomiser::~SceneRandomiser()
{

}

void SceneRandomiser::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
    if (!ros::isInitialized())
    {
        ROS_FATAL("Gazebo ROS node not initialised!");
        return;
    }

    nh_.reset(new ros::NodeHandle("randomise_plugin"));
    randomise_srv_ = nh_->advertiseService("/gazebo/randomise_scene", &SceneRandomiser::randomiseSrvCallback, this);

    ros::param::param<std::string>("/robot_name", robot_name_, "j2s7s300");

    world_ = _world;
    ROS_INFO("%sSceneRandomiser: Plugin loaded!", GREEN);
}

bool SceneRandomiser::checkCollision(ignition::math::v4::Pose3d& pose, physics::ModelPtr model, physics::ModelPtr surface)
{
    std::vector<physics::ModelPtr> models = world_->Models();

    // Create new bounding box (AABB) in the new position.
    // The current model box is used to obtain min and max values
    // for the new box
    ignition::math::v4::Box model_box = model->CollisionBoundingBox();
    ignition::math::v4::Vector3d vec1(pose.Pos().X() + (model_box.Size().X()*0.5),
                                    pose.Pos().Y() + (model_box.Size().Y()*0.5),
                                    pose.Pos().Z() + (model_box.Size().Z()*0.5));

    ignition::math::v4::Vector3d vec2(pose.Pos().X() - (model_box.Size().X()*0.5),
                                    pose.Pos().Y() - (model_box.Size().Y()*0.5),
                                    pose.Pos().Z() - (model_box.Size().Z()*0.5));

    ignition::math::v4::Box box(vec1, vec2);

    for (int i = 0; i < models.size(); i++)
    {
        if (models[i] == model ||
            models[i] == surface ||
            models[i]->GetName() == "ground_plane" ||
            models[i]->GetName() == "sun" ||
            models[i]->GetName() == robot_name_)
        { continue; }

        ignition::math::v4::Box other = models[i]->CollisionBoundingBox();

        if (box.Intersects(other))
        {
            return true;
        }
    }

    return false;
}

ignition::math::v4::Pose3d SceneRandomiser::getModelPose(const ignition::math::v4::Pose3d& centre, physics::ShapePtr shape)
{
    ignition::math::v4::Pose3d pose;
    std::random_device rseed;
    std::mt19937 rng(rseed());
    std::uniform_real_distribution<double> rot_dist(-M_PI, M_PI);

    if (shape->HasType(physics::Shape::BOX_SHAPE))
    {
        physics::BoxShape* box = static_cast<physics::BoxShape*>(shape.get());

        // the surface dimensions are reduced to avoid cases with objects
        // being on the edge
        ignition::math::v4::Vector3d dim(box->Size());
        dim.X() -= 0.2, dim.Y() -= 0.2;

        double upper_x = centre.Pos().X() + (dim.X()*0.5);
        double lower_x = centre.Pos().X() - (dim.X()*0.5);

        double upper_y = centre.Pos().Y() + (dim.Y()*0.5);
        double lower_y = centre.Pos().Y() - (dim.Y()*0.5);

        std::uniform_real_distribution<double> x_dist(lower_x, upper_x);
        std::uniform_real_distribution<double> y_dist(lower_y, upper_y);
        
        pose.Pos().Set(x_dist(rng),
                    y_dist(rng),
                    centre.Pos().Z() + (dim.Z()*0.5) + 0.05);

        // Rotation about centre of surface:
        // Translate pose by centre point in negative direction (i.e. negative of centre point)
        // Rotate pose by surface yaw
        // Translate pose again by centre of point (not negative)
        // Pose = T + Rot*(Pose + (-1*T))
        ignition::math::v4::Vector3d t = centre.Pos();
        pose.Pos() = t + centre.Rot().RotateVector(pose.Pos() + (-1*t));

        pose.Rot().Euler(0, 0, rot_dist(rng));
    }
    else if (shape->HasType(physics::Shape::CYLINDER_SHAPE))
    {
        physics::CylinderShape* cylinder = static_cast<physics::CylinderShape*>(shape.get());

        double radius = (cylinder->GetRadius() - 0.1);
        std::uniform_real_distribution<double> dist(0, 1);

        double new_r = radius * dist(rng);
        double theta = dist(rng) * 2 * M_PI;

        pose.Pos().Set(centre.Pos().X() + new_r*cos(theta),
                    centre.Pos().Y() + new_r*sin(theta),
                    centre.Pos().Z() + (cylinder->GetLength()*0.5) + 0.05);
        pose.Rot().Euler(0, 0, rot_dist(rng));
    }

    return pose;
}

bool SceneRandomiser::randomiseSrvCallback(gazebo_scene_randomiser_plugin::randomiseRequest& req, gazebo_scene_randomiser_plugin::randomiseResponse& res)
{
    std::string::size_type pos = req.surface.find("_link_");
    std::string parent_name = req.surface.substr(0, pos);
    physics::ModelPtr parent = world_->ModelByName(parent_name);

    if (!parent)
    {
        res.message = "Unable to find parent model [" + parent_name + "]";
        res.success = false;
        ROS_ERROR("[SceneRandomiser]: Unable to find parent model [%s]!", parent_name.c_str());
        return true;
    }

    std::string surface_name = req.surface.substr(pos+6);
    physics::CollisionPtr surface = parent->GetChildCollision(surface_name);

    if (!surface)
    {
        res.message = "Unable to find surface [" + surface_name + "]"; 
        res.success = false;
        ROS_ERROR("[SceneRandomiser]: Failed to find surface [%s]!", surface_name.c_str());
        return true;
    }

    physics::ShapePtr shape(surface->GetShape());
    bool supported_shape = shape->HasType(physics::Shape::BOX_SHAPE) || shape->HasType(physics::Shape::CYLINDER_SHAPE);

    if (!supported_shape)
    {
        res.message = "Surface shape not supported";
        res.success = false;
        ROS_ERROR("[SceneRandomiser]: Unsupported surface shape!");   
        return false;
    }

    std::vector<physics::ModelPtr> models = world_->Models();

    for (int i = 0; i < models.size(); i++)
    {
        physics::ModelPtr model = models[i];
        if (model->IsStatic() ||
            model == parent ||
            model->GetName() == "ground_plane" ||
            model->GetName() == "sun" ||
            model->GetName() == robot_name_)
        { continue; }

        auto start = chrono::system_clock::now();
        while (chrono::duration_cast<chrono::seconds>(chrono::system_clock::now() - start).count() <= 5)
        {
            ignition::math::v4::Pose3d pose = this->getModelPose(surface->WorldPose(), shape);

            if (!this->checkCollision(pose, model, parent))
            {
                model->SetWorldPose(pose);
                break;
            }
        }
    }

    res.message = "Scene randomised!";
    res.success = true;

    return true;
}
