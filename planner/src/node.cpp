#include "planner/controller.h"
#include "planner/planner.h"

bool checkGazeboIsUp()
{
    std::vector<std::string> nodes;
    ros::master::getNodes(nodes);

    for (int i = 0; i < nodes.size(); i++)
    {
        if (nodes[i].compare("/gazebo") == 0) return true;
    }
}

void waitForGazebo()
{
    while (!checkGazeboIsUp() && ros::ok())
    {
        ROS_WARN_THROTTLE(5, "Gazebo not detected! Waiting...");
    }

    ROS_INFO("%sGazebo is up! Starting...", GREEN);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle nh;
    waitForGazebo();
    
    Controller controller(&nh);
    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();

    return EXIT_SUCCESS;
}