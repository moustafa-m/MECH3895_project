#include <fstream>
#include <planner/start_plan.h>
#include <gazebo_scene_randomiser_plugin/randomise.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_srvs/Empty.h>
#include <boost/filesystem.hpp>
#include "planner/defines.h"

class Tester
{
public:
    Tester(ros::NodeHandle* nh)
        : nh_(*nh)
    {
        this->init();
    }

    ~Tester()
    {

    }

    void run()
    {
        int i = 1;
        while (ros::ok())
        {
            ROS_INFO("[TESTER]: Starting run #%d ...", i);

            this->runPlannerAndLog();
            this->resetArm();

            i++;
            if (i > num_runs_)
            {
                ROS_INFO("%s[TESTER]: Test runs completed!", GREEN);
                break;
            }
        }
    }

private:
    void init()
    {
        planning_client_ = nh_.serviceClient<planner::start_plan>("/start_plan", 10);
        randomiser_client_ = nh_.serviceClient<gazebo_scene_randomiser_plugin::randomise>("/gazebo/randomise_scene", 10);
        open_gripper_client_ = nh_.serviceClient<std_srvs::Empty>("/open_gripper", 10);
        init_client_ = nh_.serviceClient<std_srvs::Empty>("/go_to_init", 10);
        home_client_ = nh_.serviceClient<std_srvs::Empty>("/go_to_home", 10);

        while (!planning_client_.exists() && ros::ok())
        {
            ROS_WARN_THROTTLE(5, "planning service is not up! Waiting...");
        }

        ros::param::param<int>("~num_runs", num_runs_, 10);
        ros::param::param<std::string>("~surface", surface_, "short_table_link_surface");
        ros::param::param<std::string>("/robot_name", robot_name_, "j2s7s300");

        if (num_runs_ <= 0)
        {
            ROS_FATAL("[TESTER]: Invalid number of runs! Value set: %d", num_runs_);
            ros::shutdown();
            exit(-1);
        }
        
        auto time = std::time(nullptr);
        std::stringstream ss;
        ss << ros::package::getPath("planner") + "/test/logs/" << std::put_time(std::localtime(&time), "%b %d %Y")  << "/";
        if (!boost::filesystem::exists(ss.str()))
        {
            if (!boost::filesystem::create_directories(ss.str()))
            {
                ROS_FATAL("[TESTER]: Failed to create log directory!");
                ros::shutdown();
                exit(-1);
            }
        }

        ss << std::put_time(std::localtime(&time), "%X") << ".log";

        log_file_ = ss.str();

        ROS_INFO("%s[TESTER]: Performing %d test runs!", BLUE, num_runs_);
        ROS_INFO("%s[TESTER]: Saving log file to:\n%s", BLUE, log_file_.c_str());
        ROS_INFO("%s[TESTER]: Initialised!", GREEN);
    }

    void resetArm()
    {
        std_srvs::Empty empty_req;
        open_gripper_client_.call(empty_req);
        init_client_.call(empty_req);
    }

    void runPlannerAndLog()
    {
        planner::start_plan plan_req;
        plan_req.request.target = "cylinder";

        gazebo_scene_randomiser_plugin::randomise randomise_req;
        randomise_req.request.surface = surface_;
        if (!randomiser_client_.call(randomise_req))
        {
            ROS_FATAL("[TESTER]: Error calling randomiser service!");
            ros::shutdown();
            exit(-1);
        }

        if (planning_client_.call(plan_req))
        {
            std::fstream file;
            file.open(log_file_, std::fstream::app);
            
            // annoyingly, bool variables in ROS srv/msg are represented as uint8, so each one needs to
            // be cast to bool type to print them out correctly
            file << bool(plan_req.response.path_found) << "," << bool(plan_req.response.path_valid) << ","
                << bool(plan_req.response.grasp_success) << "," << plan_req.response.plan_time << ","
                << plan_req.response.execution_time << std::endl;

            file.flush();
            file.close();
        }
        else
        {
            ROS_FATAL("[TESTER]: Encountered error while requesting a plan!");
            ros::shutdown();
            exit(-1);
        }
    }

    ros::NodeHandle nh_;
    ros::ServiceClient planning_client_;
    ros::ServiceClient randomiser_client_;
    ros::ServiceClient open_gripper_client_;
    ros::ServiceClient init_client_;
    ros::ServiceClient home_client_;

    int num_runs_;
    std::string surface_;
    std::string robot_name_;
    std::string log_file_;

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "planner_test_node");
    ros::NodeHandle nh;

    Tester tester(&nh);
    tester.run();

    return EXIT_SUCCESS;
}
