#include <geometries_visualiser/geometries_visualiser.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "visualiser_node");
    ros::NodeHandle nh;

    GeometriesVisualiser visualiser(&nh);

    ros::Rate rate(5);
    while (ros::ok())
    {
        ROS_INFO_ONCE("%s[VisualiserNode]: Node started!", GREEN);
        visualiser.run();
        rate.sleep();
        ros::spinOnce();
    }

    std::cout << GREEN << "Exiting!" << NC << std::endl;

    return EXIT_SUCCESS;
}