#include "planner/planner.h"

Planner::Planner(ros::NodeHandle* nh, Manipulator* manip)
    : nh_(*nh), manipulator_(manip)
{
    this->init();
    this->initROS();
}

Planner::~Planner()
{

}

bool Planner::plan(trajectory_msgs::JointTrajectory& traj)
{
    planner_ = ob::PlannerPtr(new og::BFMT(si_));
    planner_->setProblemDefinition(pdef_);
    planner_->setup();

    auto t_start = HighResClk::now();
    ob::PlannerStatus solved = planner_->solve(60);
    auto t_end = HighResClk::now();

    int64_t duration = chrono::duration_cast<chrono::milliseconds>(t_end-t_start).count();
    std::cout << CYAN << "Planner took: " << duration << " ms (" << duration/1000.0 << " sec)" << std::endl;

    if (solved == ob::PlannerStatus::APPROXIMATE_SOLUTION)
    {
        ROS_WARN("Approximate solution, attempting to replan with timeout of 30 seconds!");
        t_start = HighResClk::now();
        planner_->solve(30);
        t_end = HighResClk::now();

        duration = chrono::duration_cast<chrono::milliseconds>(t_end-t_start).count();
        std::cout << CYAN << "Replanning took: " << duration << " ms (" << duration/1000.0 << " sec)" << std::endl;
    }

    plan_time_ = duration;

    if (solved == ob::PlannerStatus::EXACT_SOLUTION)
    {
        og::PathSimplifierPtr simplifier = og::PathSimplifierPtr(new og::PathSimplifier(si_));
        simplifier->simplifyMax(*pdef_->getSolutionPath()->as<og::PathGeometric>());
        
        pdef_->getSolutionPath()->as<og::PathGeometric>()->interpolate(8);
        
        std::cout << GREEN << "Solution found!\n";

        #ifdef DEBUG
        std::cout << MAGENTA << "-----\n[DEBUG]\nObtained path:\n";
        pdef_->getSolutionPath()->print(std::cout);
        std::cout << "With cost: " << pdef_->getSolutionPath()->cost(pdef_->getOptimizationObjective())
                << " and length: " << pdef_->getSolutionPath()->length() << std::endl;

        std::cout << "Path in Matrix form: \n";
        std::static_pointer_cast<og::PathGeometric>(pdef_->getSolutionPath())->printAsMatrix(std::cout);
        std::cout << "-----" << NC << std::endl;
        #endif
        this->publishMarkers();

        return this->generateTrajectory(traj);
    }
    else
    {
        std::cout << RED << "No solution found!" << NC << std::endl;
        return false;
    }
}

void Planner::setStart(const Eigen::Vector3d& start, const Eigen::Quaterniond& orientation)
{
    ob::ScopedState<ob::SE3StateSpace> start_state(space_);
    start_state->setXYZ(start[0], start[1], start[2]);
    start_state->rotation().x = orientation.x();
    start_state->rotation().y = orientation.y();
    start_state->rotation().z = orientation.z();
    start_state->rotation().w = orientation.w();
    pdef_->clearStartStates();
    pdef_->addStartState(start_state);

    #ifdef DEBUG
    std::cout << MAGENTA << "[DEBUG] Start: {" << start_state->getX() << ", " << start_state->getY() << ", " 
            << start_state->getZ() << "} {" << start_state->rotation().x << ", "
            << start_state->rotation().y << ", "
            << start_state->rotation().z << ", "
            << start_state->rotation().w << "}" << std::endl;
    #endif
}

void Planner::setGoal(const Eigen::Vector3d& goal, const Eigen::Quaterniond& orientation)
{
    ob::ScopedState<ob::SE3StateSpace> goal_state(space_);
    goal_state->setXYZ(goal[0], goal[1], goal[2]);
    goal_state->rotation().x = orientation.x();
    goal_state->rotation().y = orientation.y();
    goal_state->rotation().z = orientation.z();
    goal_state->rotation().w = orientation.w();
    pdef_->clearGoal();
    pdef_->setGoalState(goal_state);

    this->publishGoalMarker(goal);

    #ifdef DEBUG
    std::cout << MAGENTA << "[DEBUG] Goal: {" << goal_state->getX() << ", " << goal_state->getY() << ", "
            << goal_state->getZ() << "} {" << goal_state->rotation().x << ", "
            << goal_state->rotation().y << ", "
            << goal_state->rotation().z << ", "
            << goal_state->rotation().w << "}" << std::endl;
    #endif
}

void Planner::setCollisionGeometries(const std::vector<util::CollisionGeometry>& collision_boxes)
{
    collision_boxes_.clear();
    collision_boxes_ = collision_boxes;
}

void Planner::savePath()
{
    const og::PathGeometric& path = *pdef_->getSolutionPath()->as<og::PathGeometric>();
    auto time = std::time(nullptr);
    
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time), "%y-%m-%d-%X");
    std::string s = ros::package::getPath("planner") + "/paths/" + planner_->getName() + "/";

    if (!boost::filesystem::exists(s))
    {
        if (!boost::filesystem::create_directory(s))
        {
            ROS_ERROR("Unable to create directory to save path!");
            return;
        }
    }

    std::string pathtxt = s + ss.str() + ".txt";
    std::string statstxt = s + ss.str() + "_stats.txt";
    
    std::cout << GREEN << "Saving solution path to paths/" << planner_->getName() << "/" << ss.str() << ".txt\n" <<
            "Run Python or Bash scripts in paths/ to visualise" << std::endl;
    
    std::ofstream path_file, stats_file;
    path_file.open(pathtxt, std::fstream::out);
    path.printAsMatrix(path_file);
    path_file.flush();
    path_file.close();

    stats_file.open(statstxt, std::fstream::out);
    stats_file << plan_time_/1000.0 << "\t" << path.length() << "\t" << path.cost(pdef_->getOptimizationObjective());
    stats_file.flush();
    stats_file.close();

    std::string dir = ros::package::getPath("planner") + "/paths";
    std::string cmd = "cd " + dir + "&& ./plot.sh -f " + planner_->getName() + "/" + ss.str() + ".txt";
    system(cmd.c_str());
}

void Planner::clearMarkers()
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = manipulator_->getName() + "_link_base";
	marker.action = visualization_msgs::Marker::DELETEALL;
    marker_pub_.publish(marker);
}

void Planner::init()
{
    space_ = ob::StateSpacePtr(std::make_shared<ob::SE3StateSpace>());

    ob::RealVectorBounds bounds(3);
    bounds.setHigh(0, 1); bounds.setLow(0, -1); // x dimension
    bounds.setHigh(1, 1); bounds.setLow(1, -1); // y dimension
    bounds.setHigh(2, 0.9); bounds.setLow(2, 0.0); // z dimension

    space_->as<ob::SE3StateSpace>()->setBounds(bounds);
    
    // space information and validity checker
    si_ = ob::SpaceInformationPtr(new ob::SpaceInformation(space_));
    si_->setStateValidityChecker(ob::StateValidityCheckerPtr(new StateChecker(si_, manipulator_, &collision_boxes_)));
    si_->setStateValidityCheckingResolution(0.001);
    si_->setup();

    // problem definition
    pdef_ = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si_));
    pdef_->setOptimizationObjective(ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si_)));
}

void Planner::initROS()
{
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
}

bool Planner::generateTrajectory(trajectory_msgs::JointTrajectory& traj)
{
    std::vector<ob::State*> path_states = pdef_->getSolutionPath()->as<og::PathGeometric>()->getStates();
    path_states.erase(path_states.begin());

    int num_joints = manipulator_->getNumJoints();

    traj.joint_names.resize(num_joints);
    traj.points.resize(path_states.size());
    traj.joint_names = manipulator_->getJointNames();

    for (size_t i = 0; i < path_states.size(); i++)
    {
        traj.points[i].positions.resize(num_joints);
        traj.points[i].velocities.resize(num_joints);
        traj.points[i].accelerations.resize(num_joints);

        Eigen::Vector3d position;
        position << path_states[i]->as<ob::SE3StateSpace::StateType>()->getX(),
                path_states[i]->as<ob::SE3StateSpace::StateType>()->getY(),
                path_states[i]->as<ob::SE3StateSpace::StateType>()->getZ();

        Eigen::Quaterniond orientation;
        orientation.w() = path_states[i]->as<ob::SE3StateSpace::StateType>()->rotation().w;
        orientation.x() = path_states[i]->as<ob::SE3StateSpace::StateType>()->rotation().x;
        orientation.y() = path_states[i]->as<ob::SE3StateSpace::StateType>()->rotation().y;
        orientation.z() = path_states[i]->as<ob::SE3StateSpace::StateType>()->rotation().z;

        std::vector<double> angles;
        bool success = false;
        if (i == 0) { success = manipulator_->solveIK(angles, position, orientation, manipulator_->getInitPose()); }
        else { success = manipulator_->solveIK(angles, position, orientation, traj.points[i-1].positions); }

        if (success) { traj.points[i].positions = angles; }
        else { return false; }
        
        for (int j = 0; j < num_joints; j++)
        {
            traj.points[i].velocities[j] = 0;
            traj.points[i].accelerations[j] = 0;
        }
        traj.points[i].time_from_start = ros::Duration(5+(i*2));

        if (!ros::ok()) { exit(0); }
    }
    return true;
}

void Planner::publishGoalMarker(const Eigen::Vector3d& goal)
{
    visualization_msgs::Marker marker;
    marker.id = 0;
    marker.scale.x = marker.scale.y = 0.02;
    marker.color.b = marker.color.a = 1.0;
    marker.color.r = marker.color.g = 0.0;
    marker.ns = "goal";
    marker.header.frame_id = manipulator_->getName() + "_link_base";
    marker.header.stamp = ros::Time::now();
    geometry_msgs::Point p;
    p.x = goal[0];
    p.y = goal[1];
    p.z = goal[2];
    marker.points.push_back(p);
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::POINTS;
    marker_pub_.publish(marker);
}

void Planner::publishMarkers()
{
    og::PathGeometric& solution = *pdef_->getSolutionPath()->as<og::PathGeometric>();
    visualization_msgs::Marker marker;

    for (size_t i = 0; i < solution.getStateCount(); i++)
    {
        marker.id = i;
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.scale.x = 0.05, marker.scale.y = 0.01, marker.scale.z = 0.03;
        marker.color.g = marker.color.r = marker.color.a = 1.0;
        marker.color.b = 0.0;
        marker.ns = "path";
        marker.header.frame_id = manipulator_->getName() + "_link_base";
        marker.header.stamp = ros::Time::now();

        geometry_msgs::Point p;
        p.x = solution.getStates()[i]->as<ob::SE3StateSpace::StateType>()->getX();
        p.y = solution.getStates()[i]->as<ob::SE3StateSpace::StateType>()->getY();
        p.z = solution.getStates()[i]->as<ob::SE3StateSpace::StateType>()->getZ();
        marker.pose.position = p;

        Eigen::Quaterniond quat;
        quat.w() = solution.getStates()[i]->as<ob::SE3StateSpace::StateType>()->rotation().w;
        quat.x() = solution.getStates()[i]->as<ob::SE3StateSpace::StateType>()->rotation().x;
        quat.y() = solution.getStates()[i]->as<ob::SE3StateSpace::StateType>()->rotation().y;
        quat.z() = solution.getStates()[i]->as<ob::SE3StateSpace::StateType>()->rotation().z;
        // arrow is parallel to x-axis when rotation is identity but kinova end effector is parallel
        // to z-axis when at identity, so transform is needed
        quat = quat * Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitY());

        marker.pose.orientation.w = quat.w();
        marker.pose.orientation.x = quat.x();
        marker.pose.orientation.y = quat.y();
        marker.pose.orientation.z = quat.z();
        marker.points.resize(0);

        marker_pub_.publish(marker);
    }
}