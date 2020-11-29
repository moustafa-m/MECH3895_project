#pragma once

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Vector3.h>
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
        geometry_msgs::Vector3 centre;
        geometry_msgs::Vector3 dimension;
    } CollisionGeometry;
}