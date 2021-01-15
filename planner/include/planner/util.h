#pragma once

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <cmath>
#include <iostream>
#include <chrono>
#include <boost/shared_ptr.hpp>
#include "defines.h"

namespace chrono = std::chrono;

typedef chrono::high_resolution_clock HighResClk;

namespace util
{
    typedef struct CollisionGeometry
    {
        std::string name;
        geometry_msgs::Vector3 min;
        geometry_msgs::Vector3 max;
        geometry_msgs::Vector3 dimension;
        geometry_msgs::Pose pose;
    } CollisionGeometry;

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
}