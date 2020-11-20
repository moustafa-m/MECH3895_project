#include "planner/controller.h"
#include "planner/planner.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle* nh;
    
    Planner planner;
    ros::spin();

    return EXIT_SUCCESS;
}