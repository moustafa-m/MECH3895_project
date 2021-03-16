#include "planner/util.h"

util::Grid2D util::discretise2DShape(const CollisionGeometry& geom, const std::string& parent_name, const std::string& robot_name,
    const std::vector<CollisionGeometry>& objects, const double& resolution)
{
    std::vector<CollisionGeometry> surface_objs;
    for (CollisionGeometry obj : objects)
    {
        if
        (
            obj == geom ||
            obj.pose.position.z - geom.pose.position.z < 0 ||
            obj.name.find(parent_name) != std::string::npos ||
            obj.name.find(robot_name) != std::string::npos ||
            (obj.min.x > geom.max.x &&
            obj.max.x < geom.min.x &&
            obj.min.y > geom.max.y &&
            obj.max.y < geom.min.y)
        )
        { continue; }

        // std::cout << obj.name << "\n";

        surface_objs.push_back(obj);
    }
    // std::cout << "num surface obj " << surface_objs.size() << "\n";
    
    Eigen::Quaterniond surface_rot(geom.pose.orientation.w,
                            geom.pose.orientation.x,
                            geom.pose.orientation.y,
                            geom.pose.orientation.z);

    double yaw = surface_rot.toRotationMatrix().eulerAngles(0, 1, 2).z();

    Eigen::Vector2d surface_bot_left(geom.pose.position.x -geom.dimension.x*0.5,
                            geom.pose.position.y - geom.dimension.y*0.5);
    Eigen::Vector2d surface_center(geom.pose.position.x, geom.pose.position.y);

    surface_bot_left = surface_center + (Eigen::Rotation2Dd(yaw) * (surface_bot_left - surface_center));

    Grid2D result;
    result.width = std::ceil(geom.dimension.x/resolution);
    result.height = std::ceil(geom.dimension.y/resolution);
    result.resolution = resolution;
    result.origin = surface_bot_left;
    result.rotation = surface_rot;
    result.nodes.resize(result.width*result.height);

    // std::cout << "Discretising " << geom.name << "\nwidth: " << result.width << " height: " << result.height << "\n";
    // std::cout << "surface dim: " << geom.dimension.x << " " << geom.dimension.y << "\n";
    // std::cout << "bot left corner of surface: " << surface_bot_left << std::endl;

    for (int i = 0; i < result.width; i++)
    {
        for (int j = 0; j < result.height; j++)
        {
            GridNode node;

            node.size = resolution;

            node.bot_left.x() = surface_bot_left.x() + (i*resolution);
            node.bot_left.y() = surface_bot_left.y() + (j*resolution);

            node.bot_left = surface_bot_left + (Eigen::Rotation2Dd(yaw) * (node.bot_left - surface_bot_left));

            node.center.x() = node.bot_left.x() + (resolution*0.5);
            node.center.y() = node.bot_left.y() + (resolution*0.5);

            node.center = node.bot_left + (Eigen::Rotation2Dd(yaw) * (node.center - node.bot_left));

            node.orientation = surface_rot;

            // loop through all objects and check if node is occupied
            for (CollisionGeometry obj : surface_objs)
            {
                Eigen::Quaterniond quat(obj.pose.orientation.w,
                                        obj.pose.orientation.x,
                                        obj.pose.orientation.y,
                                        obj.pose.orientation.z);

                Eigen::Vector2d object_pos2d(obj.pose.position.x, obj.pose.position.y);

                // translate point to object coordinate frame i.e. origin of point becomes object center
                Eigen::Vector2d vec = node.center - object_pos2d;
                
                // rotate point in opposite direction of object rotation
                Eigen::Rotation2Dd center_rot(-1*quat.toRotationMatrix().eulerAngles(0, 1, 2).z());
                vec = center_rot*vec;

                // translate back to world frame
                vec = vec + object_pos2d;

                // node.occupied = vec.x() >= obj.min.x &&
                //                 vec.x() <= obj.max.x &&
                //                 vec.y() >= obj.min.y &&
                //                 vec.y() <= obj.max.y;

                node.occupied = vec.x() + (node.size*0.5) > obj.pose.position.x - (obj.dimension.x*0.5) &&
                                vec.x() - (node.size*0.5) < obj.pose.position.x + (obj.dimension.x*0.5) &&
                                vec.y() + (node.size*0.5) > obj.pose.position.y - (obj.dimension.y*0.5) &&
                                vec.y() - (node.size*0.5) < obj.pose.position.y + (obj.dimension.y*0.5);

                if (node.occupied) { break; }
            }

            result.nodes[i + j*result.width] = node;
        }
    }

    return result;
}
