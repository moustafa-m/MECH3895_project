#include "planner/controller.h"
#include "planner/planner.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle nh;
    
    Controller controller(&nh);
    ros::Rate rate(10);
    while (ros::ok())
    {
        rate.sleep();
        ros::spinOnce();
    }

    return EXIT_SUCCESS;
}