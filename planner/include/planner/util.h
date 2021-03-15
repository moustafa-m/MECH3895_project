#pragma once

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <cmath>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <eigen3/Eigen/Geometry>
#include "defines.h"

namespace util
{
    struct CollisionGeometry
    {
        std::string name;
        geometry_msgs::Vector3 min;
        geometry_msgs::Vector3 max;
        geometry_msgs::Vector3 dimension;
        geometry_msgs::Pose pose;
    };

    struct GridNode
    {
        // Eigen::Vector2d top_left = Eigen::Vector2d(NAN, NAN);
        Eigen::Vector2d bot_left = Eigen::Vector2d(NAN, NAN);
        Eigen::Vector2d center = Eigen::Vector2d(NAN, NAN);
        bool occupied = false;
        double size = 0.0;
        Eigen::Quaterniond orientation = Eigen::Quaterniond(NAN, NAN, NAN, NAN);
    };

    struct Grid2D
    {
        int width = 0;
        int height = 0;
        double resolution = 0;
        Eigen::Vector2d origin = Eigen::Vector2d(NAN, NAN);
        Eigen::Quaterniond rotation = Eigen::Quaterniond(NAN, NAN, NAN, NAN);
        std::vector<GridNode> nodes;
    };

    inline bool operator==(const CollisionGeometry& lhs, const CollisionGeometry& rhs)
    {
        return lhs.name.compare(rhs.name) == 0 &&
            lhs.min == rhs.min &&
            lhs.max == rhs.max &&
            lhs.pose == rhs.pose &&
            lhs.dimension == rhs.dimension;
    }

    inline bool operator!=(const CollisionGeometry& lhs, const CollisionGeometry& rhs)
    {
        return !(lhs == rhs);
    }

    template <typename T>
    T clamp(T val, T lower, T upper)
    {
        if (val < lower) return lower;
        if (val > upper) return upper;
        return val;
    }
    
    // mainly for use with floating point vectors
    template <typename T>
    bool approxEqual(std::vector<T> vec1, std::vector<T> vec2, T max_error)
    {
        assert(vec1.size() == vec2.size() && "Size mismatch! Unable to check if Vectors are approximately equal.");

        for (int i = 0; i < vec1.size(); i++)
        {
            if (std::abs(vec1[i] - vec2[i]) > max_error) return false;
        }

        return true;
    }

    Grid2D discretise2DShape(const CollisionGeometry& geom, const std::string& parent_name, const std::string& robot_name,
        const std::vector<CollisionGeometry>& objects, const double& resolution = 0.02);
}
