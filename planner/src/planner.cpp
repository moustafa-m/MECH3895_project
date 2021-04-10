#include "planner/planner.h"

Planner::Planner(ros::NodeHandle* nh)
    : nh_(*nh), manipulator_(nh), controller_(nh, &manipulator_)
{
    goal_pos_.setIdentity();
    goal_orient_.setIdentity();
    this->initROS();
    this->init();
}

Planner::~Planner()
{

}

bool Planner::plan()
{
    state_checker_->setTargetCollision(true);
    state_checker_->setNonStaticCollisions(false);
    state_checker_->setIKCheck(true);

    result_.reset();

    if (!state_checker_->isValid(pdef_->getGoal()->as<ob::GoalState>()->getState()))
    {
        std::cout << RED << "[PLANNER]: Goal state is invalid!" << NC << std::endl;
        return false;
    }

    solutions_.clear();
    planner_->clear();
    planner_->setProblemDefinition(pdef_);
    if (!planner_->isSetup()) planner_->setup();

    Timer plan_timer, exectuion_timer;

    // initial planning only computes a path without colliding with static objects and target
    plan_timer.start();
    ob::PlannerStatus solved = planner_->solve(timeout_);

    if (solved != ob::PlannerStatus::EXACT_SOLUTION)
    {
        result_.plan_time = plan_timer.elapsedMillis()/1000.0;
        std::cout << RED << "[PLANNER]: No solution found!" << NC << std::endl;
        return false;
    }

    // now check if the object is blocked by non-static objects, if it is not, then the inital obtained path is used
    std::vector<util::CollisionGeometry> blocking_objs;
    if (this->isObjectBlocked(blocking_objs))
    {
        // disable checks for collision with target and non-static objects if clutter clearing is needed
        state_checker_->setTargetCollision(false);
        state_checker_->setNonStaticCollisions(false);
        pdef_->clearSolutionPaths();

        //TODO: maybe it'd be better to put this inside planInClutter() and make a new
        // method (getAction() or smth) and call that.
        bool exit = false;
        while (ros::ok() && !exit)
        {
            // check if global timeout exceeded
            int total_time = plan_timer.elapsedMillis() + exectuion_timer.elapsedMillis();
            if (total_time >= global_timeout_*1000)
            {
                result_.partial_solution = true;
                result_.plan_time = plan_timer.elapsedMillis()/1000.0;
                result_.execution_time = exectuion_timer.elapsedMillis()/1000.0;
                ROS_ERROR("[PLANNER]: timeout exceeded!\n Time taken: %.2f", total_time/1000.0);
                return false;
            }

            // ensure timer is always started at this stage, the start() method will not
            // override previous data as it checks if the timer is already started or paused
            plan_timer.start();
            std::vector<ob::ScopedState<ob::SE3StateSpace>> states;

            solutions_.clear();

            this->update();
            this->isObjectBlocked(blocking_objs);

            Planner::ActionType action = this->planInClutter(blocking_objs, states);
            if (action == Planner::ActionType::NONE)
            {
                result_.partial_solution = true;
                result_.plan_time = plan_timer.elapsedMillis()/1000.0;
                result_.execution_time = exectuion_timer.elapsedMillis()/1000.0;
                ROS_ERROR("[PLANNER]: Failed to plan an action!");
                return false;
            }

            exit = blocking_objs.empty();

            for (int i = 0; i < states.size(); i++)
            {
                std::cout << BLUE << "Solving for state: " << i << std::endl;
                
                state_checker_->setIKCheck(true);

                // break problem into multiple sub-problems for OMPL
                ob::ProblemDefinitionPtr pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si_));
                pdef->clearGoal(); pdef->clearStartStates();

                if (i == 0) { pdef->addStartState(pdef_->getStartState(0)); }
                else { pdef->addStartState(states[i-1]); }

                pdef->setGoalState(states[i]);
                pdef->setOptimizationObjective(ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si_)));

                planner_->clear();
                planner_->setProblemDefinition(pdef);
                if (!planner_->isSetup()) planner_->setup();

                solved = planner_->solve(timeout_);

                if (solved == ob::PlannerStatus::EXACT_SOLUTION)
                {
                    og::PathSimplifierPtr simplifier = og::PathSimplifierPtr(new og::PathSimplifier(si_));
                    simplifier->simplifyMax(*pdef->getSolutionPath()->as<og::PathGeometric>());
                    pdef->getSolutionPath()->as<og::PathGeometric>()->interpolate(path_states_);
                    solutions_.push_back(*pdef->getSolutionPath()->as<og::PathGeometric>());
                    this->publishMarkers();

                    if (save_path_) { this->savePath(); }
                }
                else
                {
                    std::cout << RED << "[PLANNER]: No solution found!\nUnable to solve for state : "
                        << i << "\n" << states[i] << NC << std::endl;
                    result_.plan_time = plan_timer.elapsedMillis()/1000.0;
                    result_.execution_time = exectuion_timer.elapsedMillis()/1000.0;
                    return false;
                }
            }

            trajectory_msgs::JointTrajectory traj;
            if (!this->generateTrajectory(traj))
            {
                result_.partial_solution = true;
                result_.plan_time = plan_timer.elapsedMillis()/1000.0;
                result_.execution_time = exectuion_timer.elapsedMillis()/1000.0;
                std::cout << "[PLANNER]: Path not kinematically valid!" << std::endl;
                return false;
            }

            plan_timer.pause();

            exectuion_timer.start();
            result_.num_actions++;
            this->executeAction(action);
            exectuion_timer.pause();
        }
    }
    else
    {
        result_.num_actions++;

        // object not blocked but path may collide with non-static objects because
        // initial planning does not check for that, so path needs to be checked
        state_checker_->setNonStaticCollisions(true);
        pdef_->getSolutionPath()->as<og::PathGeometric>()->checkAndRepair(5);

        og::PathSimplifierPtr simplifier = og::PathSimplifierPtr(new og::PathSimplifier(si_));
        simplifier->simplifyMax(*pdef_->getSolutionPath()->as<og::PathGeometric>());
        pdef_->getSolutionPath()->as<og::PathGeometric>()->interpolate(path_states_);
        
        solutions_.push_back(*pdef_->getSolutionPath()->as<og::PathGeometric>());
        this->publishMarkers();

        trajectory_msgs::JointTrajectory traj;
        if (!this->generateTrajectory(traj))
        {
            result_.partial_solution = true;
            result_.plan_time = plan_timer.elapsedMillis()/1000.0;
            std::cout << "[PLANNER]: Path not kinematically valid!" << std::endl;
            return false;
        }

        plan_timer.pause();

        if (save_path_) { this->savePath(); }

        exectuion_timer.start();
        controller_.sendAction(traj);
        controller_.closeGripper();
        exectuion_timer.pause();
    }

    result_.plan_time = plan_timer.elapsedMillis()/1000.0;
    result_.execution_time = exectuion_timer.elapsedMillis()/1000.0;
    result_.path_found = true;

    std::cout << GREEN << "[PLANNER]: Solution found!\n";

    // verify grasp attempt
    std::vector<Eigen::Vector3d> positions;
    std::vector<Eigen::Quaterniond> orientations;
    manipulator_.solveFK(positions, orientations);
    this->update();
    if (std::abs(positions.back().x() - target_geom_.pose.position.x) <= 3e-2 &&
        std::abs(positions.back().y() - target_geom_.pose.position.y) <= 2e-2 &&
        std::abs(positions.back().z() - target_geom_.pose.position.z) <= 4e-2)
    {
        std::cout << GREEN << "[PLANNER]: Grasp attempt successful" << std::endl;
        result_.grasp_success = true;
    }
    else
    {
        std::cout << RED << "[PLANNER]: Grasp attempt failed!" << std::endl;
        result_.grasp_success = false;
        return false;
    }

    return true;
}

void Planner::setStart(const Eigen::Vector3d& start, const Eigen::Quaterniond& orientation)
{
    ob::ScopedState<ob::SE3StateSpace> start_state(space_);
    start_state->setXYZ(start.x(), start.y(), start.z());
    start_state->rotation().x = orientation.x();
    start_state->rotation().y = orientation.y();
    start_state->rotation().z = orientation.z();
    start_state->rotation().w = orientation.w();
    pdef_->clearStartStates();
    pdef_->addStartState(start_state);

    std::cout << BLUE << "[PLANNER]: Start pose set to:\n {"
        << start_state->getX() << ", " << start_state->getY() << ", "
        << start_state->getZ() << "} {" << start_state->rotation().x << ", "
        << start_state->rotation().y << ", " << start_state->rotation().z << ", "
        << start_state->rotation().w << "}" << std::endl;
}

void Planner::setGoal(const Eigen::Vector3d& goal, const Eigen::Quaterniond& orientation)
{
    ob::ScopedState<ob::SE3StateSpace> goal_state(space_);
    goal_state->setXYZ(goal.x(), goal.y(), goal.z());
    goal_state->rotation().x = orientation.x();
    goal_state->rotation().y = orientation.y();
    goal_state->rotation().z = orientation.z();
    goal_state->rotation().w = orientation.w();

    if (!this->verifyAndCorrectGraspPose(goal_state))
    {
        std::cout << RED << "[PLANNER]: No valid grasp poses!\n";
    }

    pdef_->clearGoal();
    pdef_->clearSolutionPaths();
    pdef_->setGoalState(goal_state);

    this->publishGoalMarker();

    goal_pos_ = Eigen::Vector3d(goal_state->getX(), goal_state->getY(), goal_state->getZ());
    goal_orient_ = Eigen::Quaterniond(goal_state->rotation().w,
                                    goal_state->rotation().x,
                                    goal_state->rotation().y,
                                    goal_state->rotation().z);

    std::cout << BLUE << "[PLANNER]: Goal pose set to:\n {"
        << goal_state->getX() << ", " << goal_state->getY() << ", "
        << goal_state->getZ() << "} {" << goal_state->rotation().x << ", "
        << goal_state->rotation().y << ", " << goal_state->rotation().z << ", "
        << goal_state->rotation().w << "}" << std::endl;
}

void Planner::setTargetGeometry(util::CollisionGeometry geom)
{
    target_geom_ = geom;
    state_checker_->setTargetName(geom.name);
}

void Planner::savePath()
{
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
    stats_file.open(statstxt, std::fstream::out);
    std::stringstream path_stream;
    for (int i = 0; i < solutions_.size(); i++)
    {
        solutions_[i].printAsMatrix(path_stream);
        stats_file << result_.plan_time/1000.0 << "\t" << solutions_[i].length() << "\t"
            << solutions_[i].cost(pdef_->getOptimizationObjective()) << std::endl;
    }
    
    // the paths contain an empty line at the end which makes the graph in gnuplot plot get drawn incorrectly
    // so they need to be not included
    std::string line;
    while (getline(path_stream, line))
    {
        if (!line.empty()) { path_file << line << "\n"; }
    }

    path_file.flush();
    path_file.close();

    stats_file.flush();
    stats_file.close();
}

void Planner::clearMarkers()
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = manipulator_.getName() + "_link_base";
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
    state_checker_ = std::make_shared<StateChecker>(StateChecker(si_, &manipulator_, &collision_boxes_));
    si_->setStateValidityChecker(ob::StateValidityCheckerPtr(state_checker_));
    si_->setStateValidityCheckingResolution(0.001);
    si_->setup();

    // problem definition
    pdef_ = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si_));
    pdef_->setOptimizationObjective(ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si_)));

    if (planner_name_ == "RRTStar") { planner_ = ob::PlannerPtr(new og::RRTstar(si_)); }
    else if (planner_name_ == "BFMT") { planner_ = ob::PlannerPtr(new og::BFMT(si_)); }
    else { planner_ = ob::PlannerPtr(new og::KPIECE1(si_)); planner_->setup(); }

    ROS_INFO("%s[PLANNER]: Initialised!", GREEN);
}

void Planner::initROS()
{
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
    grid_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/surface_grid", 10);
    models_sub_ = nh_.subscribe("/gazebo/model_states", 10, &Planner::modelStatesCallback, this);
    start_plan_srv_ = nh_.advertiseService("/start_plan", &Planner::startPlanSrvCallback, this);
    reset_arm_srv_ = nh_.advertiseService("/reset_arm", &Planner::resetArmSrvCallback, this);
    collisions_client_ = nh_.serviceClient<gazebo_geometries_plugin::geometry>("/gazebo/get_geometry");

    ros::param::param<std::vector<std::string>>("/gazebo/static_objects", static_objs_, {"INVALID"});
    ros::param::param<std::string>("/gazebo/surface", surface_geom_.name, "small_table_link_surface");
    ros::param::param<std::string>("~planner/name", planner_name_, "KPIECE1");
    ros::param::param<int>("~planner/global_timeout", global_timeout_, 300);
    ros::param::param<int>("~planner/timeout", timeout_, 60);
    ros::param::param<int>("~planner/path_states", path_states_, 10);
    ros::param::param<bool>("~planner/save_path", save_path_, false);

    std::size_t pos = surface_geom_.name.find("link_");
    surface_parent_name_ = surface_geom_.name.substr(0, pos);
    // part with "link_" is not needed due to the geometries plugin not including it in the returned variables
    surface_geom_.name.erase(pos, 5);

    if (planner_name_ != "KPIECE1" && planner_name_ != "BFMT" && planner_name_ != "RRTStar")
    {
        ROS_WARN("[PLANNER]: unrecognised planner name [%s], setting to KPIECE1!", planner_name_.c_str());
        planner_name_ = "KPIECE1";
        ros::param::set("~planner/name", "KPIECE1");
    }
    
    ROS_INFO("%s*******PLANNER PARAMS********", BLUE);
    
    std::stringstream ss;
    for (int i = 0; i < static_objs_.size(); i++) { ss << static_objs_[i] << " "; }
    ROS_INFO_STREAM(BLUE << "Static Objs\t: " << ss.str());
    ROS_INFO_STREAM(BLUE << "surface\t\t: " << surface_geom_.name);
    ROS_INFO_STREAM(BLUE << "name\t\t: " << planner_name_);
    ROS_INFO_STREAM(BLUE << "global_timeout\t: " << global_timeout_);
    ROS_INFO_STREAM(BLUE << "timeout\t\t: " << timeout_);
    ROS_INFO_STREAM(BLUE << "path_states\t: " << path_states_);
    ROS_INFO_STREAM(BLUE << "save_path\t: " << std::boolalpha << save_path_);
    
    ROS_INFO("%s*****************************", BLUE);

    while (models_sub_.getNumPublishers() == 0 && ros::ok())
    {
        ROS_INFO_DELAYED_THROTTLE(5, "%s[PLANNER]: Waiting for Gazebo topics to come up...", CYAN);
    }
}

bool Planner::generateTrajectory(trajectory_msgs::JointTrajectory& traj)
{
    std::vector<ob::State*> path_states = solutions_[0].getStates();
    path_states.erase(path_states.begin());

    for (int i = 1; i < solutions_.size(); i++)
    {
        std::vector<ob::State*> solution_states = solutions_[i].getStates();
        solution_states.erase(solution_states.begin());
        path_states.reserve(path_states.size() + solution_states.size());
        path_states.insert(path_states.end(), solution_states.begin(), solution_states.end());
    }

    int num_joints = manipulator_.getNumJoints();

    traj.joint_names.resize(num_joints);
    traj.points.resize(path_states.size());
    traj.joint_names = manipulator_.getJointNames();

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
        if (i == 0)
        {
            // This step ensures this will work for online planning.
            // The initial state of the arm is used directly from the joint_states topic
            // that is subscribed to in the Manipulator class rather than assuming the arm has
            // started from its init pose which is not always the case for online planning.
            std::vector<double> current_angles = manipulator_.getJointStates().position;
            for (int i = 0; i < manipulator_.getFingerNames().size(); i++) { current_angles.pop_back(); }
            success = manipulator_.solveIK(angles, position, orientation, current_angles);
        }
        else
        {
            success = manipulator_.solveIK(angles, position, orientation, traj.points[i-1].positions);
        }

        if (!success)
        {
            std::cout << RED << "[PLANNER]: IK solver failed for: \npos: { " << position
                << " }\nquat: { " << orientation.coeffs() << " }" << std::endl;
            return false;
        }
        
        traj.points[i].positions = angles;
        
        for (int j = 0; j < num_joints; j++)
        {
            traj.points[i].velocities[j] = 0;
            traj.points[i].accelerations[j] = 0;
        }
        traj.points[i].time_from_start = ros::Duration((i+1));

        if (!ros::ok()) { exit(0); }
    }
    return true;
}

void Planner::publishGoalMarker()
{
    visualization_msgs::Marker marker;
    if (!target_geom_.name.empty())
    {
        marker.id = 0;
        marker.scale = target_geom_.dimension;
        marker.pose = target_geom_.pose;
        marker.color.b = marker.color.a = 1.0;
        marker.color.r = marker.color.g = 0.0;
        marker.ns = "goal";
        marker.header.frame_id = manipulator_.getName() + "_link_base";
        marker.header.stamp = ros::Time::now();
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::CUBE;
        marker_pub_.publish(marker);
    }

    marker.id = 1;
    marker.ns = "goal";
    marker.header.frame_id = manipulator_.getName() + "_link_base";
    marker.header.stamp = ros::Time::now();
    marker.color.a = 1.0; marker.color.b = 1.0; marker.color.r = 0.3; marker.color.g = 0;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::ARROW;
    
    marker.scale.x = 0.1, marker.scale.y = 0.01, marker.scale.z = 0.05;
    marker.pose.position.x = goal_pos_.x();
    marker.pose.position.y = goal_pos_.y();
    marker.pose.position.z = goal_pos_.z();

    Eigen::Quaterniond quat = goal_orient_ * Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitY());
    marker.pose.orientation.w = quat.w();
    marker.pose.orientation.x = quat.x();
    marker.pose.orientation.y = quat.y();
    marker.pose.orientation.z = quat.z();

    marker.points.resize(0);
    marker_pub_.publish(marker);
}

void Planner::publishGridMap()
{
    nav_msgs::OccupancyGrid grid;
    grid.info.width = surface_grid_.width;
    grid.info.height = surface_grid_.height;
    grid.info.resolution = surface_grid_.resolution;

    grid.info.origin.position.x = surface_grid_.origin.x();
    grid.info.origin.position.y = surface_grid_.origin.y();
    grid.info.origin.position.z = surface_geom_.pose.position.z + surface_geom_.dimension.z*1.5;
    grid.info.origin.orientation = surface_geom_.pose.orientation;

    grid.info.map_load_time = ros::Time::now();

    grid.header.frame_id = manipulator_.getName() + "_link_base";
    grid.header.stamp = ros::Time::now();
    grid.header.seq += 1;

    grid.data.resize(grid.info.width*grid.info.height);

    for (int i = 0; i < surface_grid_.width; i++)
    {
        for (int j = 0; j < surface_grid_.height; j++)
        {
            grid.data[i + j*surface_grid_.width] = 100*(surface_grid_.nodes[i + j*surface_grid_.width].occupied);
        }
    }
    grid_pub_.publish(grid);
}

void Planner::publishMarkers()
{
    int marker_id = 0;

    // RGB format
    const double colours[4][3] = { {1.0, 1.0, 0.0}, {1.0, 0.0, 1.0}, {0.0, 1.0, 1.0}, {0.0, 1.0, 0.0} }; //TODO: better way to colour the paths
    for (int j = 0; j < solutions_.size(); j++)
    {
        visualization_msgs::Marker marker;

        // ---> state markers
        marker.id = j;
        marker.ns = "states";
        marker.header.frame_id = manipulator_.getName() + "_link_base";
        marker.header.stamp = ros::Time::now();
        marker.color.a = 1.0; marker.color.g = 1.0; marker.color.r = marker.color.g = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = marker.scale.y = marker.scale.z = 0.02;
        marker.pose.position.x = solutions_[j].getStates()[0]->as<ob::SE3StateSpace::StateType>()->getX();
        marker.pose.position.y = solutions_[j].getStates()[0]->as<ob::SE3StateSpace::StateType>()->getY();
        marker.pose.position.z = solutions_[j].getStates()[0]->as<ob::SE3StateSpace::StateType>()->getZ();
        marker.pose.orientation.w = 1;
        marker.pose.orientation.x = marker.pose.orientation.y = marker.pose.orientation.z = 0;
        marker.points.resize(0);
        marker_pub_.publish(marker);
        // <--- state markers

        // ---> path markers
        for (size_t i = 0; i < solutions_[j].getStateCount(); i++)
        {
            marker.id = marker_id++;
            marker.action = visualization_msgs::Marker::ADD;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.scale.x = 0.05, marker.scale.y = 0.01, marker.scale.z = 0.03;

            int colour_row = j * 123456789 % 4;
            // marker.color.g = marker.color.r = ((float)j)/solutions_.size() , marker.color.a = 1.0;
            // marker.color.b = 0.0;
            marker.color.r = colours[colour_row][0];
            marker.color.g = colours[colour_row][1];
            marker.color.b = colours[colour_row][2];
            marker.color.a = 1.0;

            marker.ns = "path";
            marker.header.frame_id = manipulator_.getName() + "_link_base";
            marker.header.stamp = ros::Time::now();

            geometry_msgs::Point p;
            p.x = solutions_[j].getStates()[i]->as<ob::SE3StateSpace::StateType>()->getX();
            p.y = solutions_[j].getStates()[i]->as<ob::SE3StateSpace::StateType>()->getY();
            p.z = solutions_[j].getStates()[i]->as<ob::SE3StateSpace::StateType>()->getZ();
            marker.pose.position = p;

            Eigen::Quaterniond quat;
            quat.w() = solutions_[j].getStates()[i]->as<ob::SE3StateSpace::StateType>()->rotation().w;
            quat.x() = solutions_[j].getStates()[i]->as<ob::SE3StateSpace::StateType>()->rotation().x;
            quat.y() = solutions_[j].getStates()[i]->as<ob::SE3StateSpace::StateType>()->rotation().y;
            quat.z() = solutions_[j].getStates()[i]->as<ob::SE3StateSpace::StateType>()->rotation().z;
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
        // <--- path markers
    }
}

void Planner::modelStatesCallback(gazebo_msgs::ModelStatesConstPtr msg)
{
    models_ = msg;
}

void Planner::update()
{
    this->clearMarkers();

    collision_boxes_.clear();

    for (int i = 0; i < models_->name.size(); i++)
    {
        if (models_->name[i].find(manipulator_.getName()) != std::string::npos) continue;

        gazebo_geometries_plugin::geometry srv;
        srv.request.model_name = models_->name[i];
        if (collisions_client_.call(srv))
        {
            if (!srv.response.success)
            {
                ROS_ERROR("[PLANNER]: Geometries plugin returned an error for [%s]!", models_->name[i].c_str());
                continue;
            }

            for (size_t j = 0; j < srv.response.name.size(); j++)
            {
                util::CollisionGeometry temp;
                temp.name = srv.response.name[j];
                temp.pose = srv.response.pose[j];
                temp.min = srv.response.min_bounds[j];
                temp.max = srv.response.max_bounds[j];
                temp.dimension = srv.response.dimensions[j];
                
                collision_boxes_.push_back(temp);

                if (temp.name == target_geom_.name) { this->setTargetGeometry(temp); }
                else if (temp.name == surface_geom_.name) { surface_geom_ = temp; }
            }
        }
    }

    int num_joints = manipulator_.getNumJoints();
    // collision geometries for Kinova links and fingers
    for (int i = 0; i < num_joints+3; i++)
    {
        std::string link;
        if (i < manipulator_.getNumJoints()) { link = "_link_" + std::to_string(i+1); }
        else { link = "_link_finger_" + std::to_string((i+1)-num_joints); }

        gazebo_geometries_plugin::geometry srv;
        srv.request.model_name = manipulator_.getName() + link;
        if (collisions_client_.call(srv))
        {
            if (!srv.response.success)
            {
                ROS_ERROR("[PLANNER]: Geometries plugin returned an error for [%s]!", srv.request.model_name.c_str());
                continue;
            }

            for (size_t j = 0; j < srv.response.name.size(); j++)
            {
                util::CollisionGeometry temp;
                temp.name = srv.response.name[j];
                temp.pose = srv.response.pose[j];
                temp.min = srv.response.min_bounds[j];
                temp.max = srv.response.max_bounds[j];
                temp.dimension = srv.response.dimensions[j];
                collision_boxes_.push_back(temp);
            }
        }
    }

    surface_grid_ = util::discretise2DShape(surface_geom_, surface_parent_name_, manipulator_.getName(), collision_boxes_, 0.01);
    this->publishGridMap();

    std::vector<Eigen::Vector3d> manip_positions;
    std::vector<Eigen::Quaterniond> manip_quats;
    manipulator_.solveFK(manip_positions, manip_quats);
    this->setStart(manip_positions.back(), manip_quats.back());

    goal_pos_ = Eigen::Vector3d(target_geom_.pose.position.x,
                                target_geom_.pose.position.y,
                                target_geom_.pose.position.z);
    if (goal_orient_.coeffs() == Eigen::Quaterniond::Identity().coeffs()) goal_orient_ = manip_quats.back();

    this->setGoal(goal_pos_, goal_orient_);
}

bool Planner::verifyAndCorrectGraspPose(ob::ScopedState<ob::SE3StateSpace>& state)
{
    state_checker_->setIKCheck(true);
    state_checker_->setNonStaticCollisions(false);
    state_checker_->setTargetCollision(true);

    if (state_checker_->isValid(state.get())) return true;

    ROS_WARN("[PLANNER]: Grasp pose invalid, attempting to obtain a new one!");

    ob::ScopedState<ob::SE3StateSpace> goal_state(space_);
    goal_state = state;

    Eigen::Quaterniond quat(goal_state->rotation().w,
                            goal_state->rotation().x,
                            goal_state->rotation().y,
                            goal_state->rotation().z);
    int num_iterations = 500;
    for (int i = 1; i <= num_iterations; i++)
    {
        for (int j = 0; j <= 1; j++)
        {
            int dir = (j == 0) ? -1 : 1;
            double rot_val = M_PI_2*double(dir)*i/num_iterations;

            Eigen::Quaterniond new_quat = Eigen::AngleAxisd(rot_val, Eigen::Vector3d::UnitZ()) * quat;

            // if (i % 100 == 0)
            // {
            //     goal_orient_ = new_quat;
            //     this->publishGoalMarker();
            //     sleep(2);
            // }

            goal_state->rotation().x = new_quat.x();
            goal_state->rotation().y = new_quat.y();
            goal_state->rotation().z = new_quat.z();
            goal_state->rotation().w = new_quat.w();

            if (state_checker_->isValid(goal_state.get()))
            {
                state = goal_state;
                goal_orient_ = new_quat;
                std::cout << GREEN << "[PLANNER]: found valid grasp pose!\n";
                return true;
            }
        }
    }

    std::cout << RED << "[PLANNER]: Failed to find valid grasp pose!\n";
    return false;
}

bool Planner::isObjectBlocked(std::vector<util::CollisionGeometry>& objs)
{
    objs.clear();
    bool clear = true;

    ob::ScopedState<ob::SE3StateSpace> start(space_);
    start = pdef_->getStartState(0);

    std::vector<int> idxs;

    for (size_t i = 0; i < collision_boxes_.size(); i++)
    {
        bool is_static = std::any_of(static_objs_.begin(), static_objs_.end(), [this, i](const std::string& str)
            { return collision_boxes_[i].name.find(str) != std::string::npos; });

        bool out_of_workspace = std::abs(collision_boxes_[i].pose.position.x) > std::abs(goal_pos_.x()) + 0.1 ||
            std::abs(collision_boxes_[i].pose.position.x) < std::abs(start->getX()) ||
            std::abs(collision_boxes_[i].pose.position.y - goal_pos_.y()) > 0.3 ||
            std::abs(collision_boxes_[i].pose.position.z - goal_pos_.z()) > 0.2;
        
        if (is_static || out_of_workspace ||
            collision_boxes_[i].name.find(manipulator_.getName()) != std::string::npos ||
            collision_boxes_[i].name.find(target_geom_.name) != std::string::npos)
        { continue; }

        if (std::abs(collision_boxes_[i].pose.position.y - goal_pos_.y()) < 0.08)
        {
            double min_clearance = target_geom_.dimension.x/2;
            // If the object x relative to the target x is more than or equal the clearance
            // then the object will need to be moved away.
            // This assumes that it is not possible for the difference to be below this
            // clearance, since anything below this clearance will collide/penetrate the target.
            if (std::abs(goal_pos_.x()) - std::abs(collision_boxes_[i].pose.position.x) >= min_clearance) idxs.push_back(i);
            clear = false;
        }
    }

    if (!clear)
    {
        if (idxs.empty())
        {
            std::cout << CYAN << "[PLANNER]: Object is blocked" << NC << std::endl;
        }
        else
        {
            std::cout << CYAN << "[PLANNER]: Path blocked, found " << idxs.size() << " objects to clear" << NC << std::endl;
            
            std::cout << CYAN << "[PLANNER]: { ";
            for (int i = 0; i < idxs.size(); i++)
            {
                int index = idxs[i];
                std::cout << collision_boxes_[index].name << " ";
                objs.push_back(collision_boxes_[index]);
            }
            std::cout << "}" << std::endl;

            // sort by increasing x values
            std::sort(objs.begin(), objs.end(), [](const util::CollisionGeometry& lhs, const util::CollisionGeometry& rhs)
                { return std::abs(lhs.pose.position.x) < std::abs(rhs.pose.position.x); });
        }
    }

    return !clear;
}

//TODO: might be worth to put the path planning and action planning bits together in here.
// This will require splitting this up however.
Planner::ActionType Planner::planInClutter(const std::vector<util::CollisionGeometry>& objs,
    std::vector<ob::ScopedState<ob::SE3StateSpace>>& states)
{
    ob::ScopedState<ob::SE3StateSpace> state(space_);
    state = pdef_->getStartState(0);

    bool should_init = std::abs(state->getY() - goal_pos_.y()) > 0.03;
    if (should_init)
    {
        double init_x = (objs.empty()) ? goal_pos_.x()*0.60 : objs[0].pose.position.x*0.60;
        double init_y = goal_pos_.y();
        double init_z = goal_pos_.z();
        state->setXYZ(init_x, init_y, init_z);

        if (state_checker_->isValid(state.get()))
        {
            states.push_back(state);
        }
        else if (std::abs(state->getY() - init_y*0.8) > 0.03)
        {
            // if not valid, most likely that object is close to a wall, so, offset end-effector a bit
            state->setY(init_y*0.8);
            states.push_back(state);
        }
    }

    if (!objs.empty())
    {
        bool success = this->getPushAction(states, objs, objs.front());

        if (success)
        {
            return ActionType::PUSH;
        }
        else
        {
            success = this->getGraspAction(states, objs.front());
            return (success) ? ActionType::GRASP : ActionType::NONE;
        }
    }
    else
    {
        if (!this->getPushGraspAction(target_geom_, state)) { return ActionType::NONE; }
        states.push_back(state);

        return ActionType::PUSH_GRASP;
    }
}

bool Planner::getPushAction(std::vector<ob::ScopedState<ob::SE3StateSpace>>& states, const std::vector<util::CollisionGeometry>& objs,
    const util::CollisionGeometry& geom)
{
    if (std::abs(goal_pos_.x()) - std::abs(geom.pose.position.x) <= 0.07)
    {
        std::cout << RED << "[PLANNER]: object to be pushed is too close to target!" << std::endl;
        return false;
    }

    state_checker_->setIKCheck(true);
    double desired_x = geom.pose.position.x;
    double desired_y = geom.pose.position.y;
    double init_dist = geom.dimension.y + 0.05;

    util::CollisionGeometry new_geom = geom;

    // Checks for objects that are close to the object that will be pushed.
    // These objects can be pushed using one action.
    for (int i = 0; i < objs.size(); i++)
    {
        if (objs[i] == geom) continue;
        bool intersect = (geom.min.x < objs[i].max.x) && (geom.max.x > objs[i].min.x);

        if (intersect)
        {
            init_dist += objs[i].dimension.y;
            desired_x = (geom.pose.position.x + objs[i].pose.position.x)/2;
            desired_y = (geom.pose.position.y + objs[i].pose.position.y)/2;
            
            new_geom.pose.position.x = desired_x;
            new_geom.pose.position.y = desired_y;

            new_geom.dimension.y += std::abs(new_geom.pose.position.y - objs[i].pose.position.y)
                                    + 0.5*(new_geom.dimension.y+objs[i].dimension.y);
            new_geom.dimension.x += std::abs(new_geom.pose.position.x - objs[i].pose.position.x)
                                    + 0.5*(new_geom.dimension.x+objs[i].dimension.x);

            new_geom.min.y = new_geom.pose.position.y - (new_geom.dimension.y*0.5);
            new_geom.max.y = new_geom.pose.position.y + (new_geom.dimension.y*0.5);

            new_geom.min.x = new_geom.pose.position.x - (new_geom.dimension.x*0.5);
            new_geom.max.x = new_geom.pose.position.x + (new_geom.dimension.x*0.5);

            std::cout << BLUE << "[PLANNER]: merging " << objs[i].name << " with " << geom.name
                << "\nNew init dist: " << init_dist << "m new dim.y: " << new_geom.dimension.y
                << "m" << NC << std::endl;
        }
    }

    int direction = (geom.pose.position.y > target_geom_.pose.position.y) ? 1 : -1;
    // initially move to be directly beside blocking object (based on direction)
    desired_y = desired_y - direction*init_dist;

    ob::ScopedState<ob::SE3StateSpace> init_state(space_);

    init_state = pdef_->getStartState(0);
    init_state->setX(desired_x);
    init_state->setY(desired_y);
    init_state->setZ(goal_pos_.z());

    if (!state_checker_->isValid(init_state.get()))
    {
        std::cout << RED << "[PLANNER]: Failed to get push action" << std::endl;
        return false;
    }

    ob::ScopedState<ob::SE3StateSpace> push_state(space_);
    push_state = init_state;

    double clearance = 0.1 - std::abs(geom.pose.position.y - target_geom_.pose.position.y);
    // y coord of the object's corner
    double geom_extreme_ycoord = geom.pose.position.y - direction*geom.dimension.y*0.5;
    double push_dist = clearance + std::abs(desired_y - geom_extreme_ycoord);
    push_dist -= 0.03; // subtract 3cm to account for the fingers making contact, not the center of the end-effector frame

    // push action
    push_state->setY(desired_y + direction*(push_dist));

    // propagate the arm and object state along the path and check if is valid
    for (int i = 1; i <= 10; i++)
    {
        // object
        double geom_y_pos = new_geom.pose.position.y + direction * (clearance*i/10);

        for (int j = 0; j < collision_boxes_.size(); j++)
        {
            bool is_static = std::any_of(static_objs_.begin(), static_objs_.end(), [this, j](const std::string& str)
                { return collision_boxes_[j].name.find(str) != std::string::npos; });
            if (!is_static) continue;

            double max_y, min_y;
            min_y = geom_y_pos - (new_geom.dimension.y*0.5);
            max_y = geom_y_pos + (new_geom.dimension.y*0.5);

            bool collide = (new_geom.min.x < collision_boxes_[j].max.x) && (new_geom.max.x > collision_boxes_[j].min.x) &&
                (min_y < collision_boxes_[j].max.y) && (max_y > collision_boxes_[j].min.y) &&
                (new_geom.min.z + 0.03 < collision_boxes_[j].max.z) && (new_geom.max.z - 0.03 > collision_boxes_[j].min.z);
            if (collide)
            {
                std::cout << RED << "[PLANNER]: object collides with " << collision_boxes_[j].name << " if pushed!"
                    << std::endl;
                return false;
            }
        }

        // arm
        ob::ScopedState<ob::SE3StateSpace> state(space_);
        state = push_state;
        state->setY(desired_y + direction * (push_dist*i/10));
        if (!state_checker_->isValid(state.get()))
        {
            std::cout << RED << "[PLANNER]: Push action not possible!" << std::endl;
            return false;
        }
    }

    states.push_back(init_state);
    states.push_back(push_state);

    // reset
    direction = (goal_pos_.x() < 0) ? 1 : -1;
    push_state->setX(geom.pose.position.x + (direction*0.1));
    push_state->setY(goal_pos_.y());
    states.push_back(push_state);

    return true;
}

bool Planner::getGraspAction(std::vector<ob::ScopedState<ob::SE3StateSpace>>& states, const util::CollisionGeometry& geom)
{
    Eigen::Vector3d dim(geom.dimension.x, geom.dimension.y, geom.dimension.z);
    Eigen::Quaterniond rot(geom.pose.orientation.w,
                        geom.pose.orientation.x,
                        geom.pose.orientation.y,
                        geom.pose.orientation.z);

    dim = rot * dim;
    if (dim.y() >= 0.075)
    {
        std::cout << RED << "[PLANNER]: Object too large for grasping! y-axis Dimension: " << dim.y()
            << " (" << geom.dimension.y << " without rot)" << std::endl;
        return false;
    }

    ob::ScopedState<ob::SE3StateSpace> grasp_state(space_), relocate_state(space_), start(space_);
    grasp_state = relocate_state = start = pdef_->getStartState(0);
    grasp_state->setXYZ(geom.pose.position.x, geom.pose.position.y, goal_pos_.z());

    if (!this->verifyAndCorrectGraspPose(grasp_state))
    {
        std::cout << RED << "[PLANNER]: " << geom.name << " cannot be grasped!" << std::endl;
        return false;
    }

    // ---> Helper functions
    // checks if the indices are within the grid's bounds
    auto isInside = [&, this](int col, int row)
    {
        return col > 0 && col < surface_grid_.width && row > 0 && row < surface_grid_.height;
    };

    // returns a pair, first element is the number of closeby objects, second is the distance to the closest object
    auto getNumObjClose = [&, this](const Eigen::Vector2d& pos, const double& thresh = 0.15)
    {
        int num_close_objects = 0;
        double least_dist = std::numeric_limits<double>::infinity();
        for (int i = 0; i < collision_boxes_.size(); i++)
        {   
            if (surface_geom_ == collision_boxes_[i] || geom == collision_boxes_[i]) { continue; }

            Eigen::Vector2d obj_pos(collision_boxes_[i].pose.position.x,
                                    collision_boxes_[i].pose.position.y);
            
            Eigen::Vector2d diff = pos - obj_pos;
            if (diff.norm() <= thresh) { num_close_objects++; }
            if (diff.norm() <= least_dist) { least_dist = diff.norm(); }
        }

        return std::pair<int, double>{num_close_objects, least_dist};
    };
    // <---Helper functions 

    // ---> variables
    // f(x) = w1*num_neighbours + w2*num_objs + w3*obj_dist + w4*travel_dist, where w1, w2, w3 & w4 are weights
    // Objective is to find the grid cell with the highest value for f(x)
    // num_neighbours is the number of unoccupied neighbouring cells
    // num_objects is number of objects close by to the cell for which f(x) is being computed
    // obj_dist is the distance to the closest object
    // travel_dist is the direct distance between the initial and final states
    double w1 = 2, w2 = -2, w3 = 5, w4 = -50;
    double val, best_val;
    val = best_val = -1*std::numeric_limits<double>::infinity();

    ob::ScopedState<ob::SE3StateSpace> ditto_state(space_);
    ditto_state = grasp_state;
    
    Eigen::Vector2d best_pos(NAN, NAN);
    Eigen::Vector2d target_pos(target_geom_.pose.position.x,
                            target_geom_.pose.position.y);

    std::vector<std::pair<int, int>> directions = { {1,0}, {-1,0}, {0,1}, {0,-1} };
    
    bool found_state = false;
    // <--- variables
    
    // This performs a limited BFS on all the grid nodes to find a node that has the highest value for f(x)
    // ---> start search
    for (int i = 0; i < surface_grid_.width; i++)
    {
        for (int j = 0; j < surface_grid_.height; j++)
        {
            util::GridNode node = surface_grid_.nodes[i + j*surface_grid_.width];

            if (node.occupied) { continue; }
            
            ditto_state->setX(node.center.x());
            ditto_state->setY(node.center.y());
            
            Eigen::Vector2d target_diff = node.center - target_pos;
            
            // check some conditions before deciding if BFS will be performed on this node.
            // the distance conditions ensure the arm stays within a certain region relative
            // to the target object and the grasped object. This makes it so that there is
            // less likelihood of causing unwanted changes to the scene that affect the target.
            if (state_checker_->isValid(ditto_state.get()) &&
                std::abs(target_diff.y()) >= 0.15 &&
                std::abs(geom.pose.position.x) - std::abs(node.center.x()) >= 0)
            {
                // ---> start BFS
                // std::cout << "BFS for " << node.center << "\n\n";
                std::vector<bool> visited(surface_grid_.nodes.size());
                std::queue<std::pair<int, int>> queue;
                queue.push({i, j});
                visited[i + j*surface_grid_.width] = true;

                Eigen::Vector2d center;
                center = node.center;

                int num_good_neighbours = 0;

                while (!queue.empty())
                {
                    std::pair<int, int> point = queue.front();
                    queue.pop();

                    util::GridNode curr_node = surface_grid_.nodes[point.first + point.second*surface_grid_.width];
                    for (std::pair<int, int> dir : directions)
                    {
                        int new_x = point.first + dir.first;
                        int new_y = point.second + dir.second;
                        
                        if (!isInside(new_x, new_y)) { continue; }

                        util::GridNode new_node = surface_grid_.nodes[new_x + new_y*surface_grid_.width];
                        Eigen::Vector2d dist_vec = new_node.center - node.center;

                        if (!(visited[new_x + new_y*surface_grid_.width] || new_node.occupied) && dist_vec.norm() <= 0.06)
                        {
                            // std::cout << new_node.center << " is valid: " << dist_vec.norm() << "\n\n";
                            queue.push({new_x, new_y});
                            num_good_neighbours++;
                            visited[new_x + new_y*surface_grid_.width] = true;
                            center = 0.5*(curr_node.center + new_node.center);
                        }
                    }
                }
                // <--- end BFS
                
                // take initial state as the grasp state which is equivalent to the grasp target position
                Eigen::Vector2d grasp_target_pos(geom.pose.position.x, geom.pose.position.y);
                double dist = (center - grasp_target_pos).norm();

                std::pair<int, double> num_close_objects = getNumObjClose(center);

                // evaluate node using f(x)
                val = w1*num_good_neighbours + w2*num_close_objects.first + w3*num_close_objects.second + w4*dist;
                // std::cout << "\n----------\n\n";

                ditto_state->setX(center.x());
                ditto_state->setY(center.y());
                if (val > best_val && state_checker_->isValid(ditto_state.get()))
                {
                    best_val = val;
                    best_pos = center;
                    found_state = true;
                }
            }
        }
    }
    // <--- end search

    if (found_state)
    {
        std::cout << GREEN << "[PLANNER]: Found position to relocate object!\npos: "
            << best_pos.format(Eigen::IOFormat(3, Eigen::DontAlignCols, ", ", ", ")) << NC << std::endl;
        
        relocate_state->setXYZ(best_pos.x(), best_pos.y(), grasp_state->getZ());
        states.push_back(grasp_state);
        states.push_back(relocate_state);
        relocate_state->setX(best_pos.x()*0.7);
        states.push_back(relocate_state);
        return true;
    }
    else
    {
        // if the above method fails, just get a random state to drop the object

        ROS_WARN("[PLANNER]: Failed to find a position to relocate, attempting to sample a random state!");

        double x_mean = geom.pose.position.x*0.3;
        double y_mean = geom.pose.position.y;
        std::uniform_real_distribution<double> x_dist(x_mean-0.10, x_mean+0.10);
        std::uniform_real_distribution<double> y_dist(y_mean-0.15, y_mean+0.15);
        std::random_device rseed;
        std::mt19937 rng(rseed());
        Timer timer;
        timer.start();

        while (timer.elapsedMillis() <= 5000)
        {
            relocate_state->setXYZ(x_dist(rng), y_dist(rng), start->getZ());
            if (state_checker_->isValid(relocate_state.get()))
            {
                found_state = true;
                states.push_back(grasp_state);
                states.push_back(relocate_state);
                timer.reset();
                break;
            }
        }

        if (!found_state)
        {
            std::cout << RED << "[PLANNER]: Failed to plan grasp action!" << std::endl;
            return false;
        }
    }

    ROS_WARN("[PLANNER]: Successfully sampled a state, object will be dropped!");

    return true;
}

bool Planner::getPushGraspAction(const util::CollisionGeometry& geom, ob::ScopedState<ob::SE3StateSpace>& state)
{
    Eigen::Vector3d pos(geom.pose.position.x,
                        geom.pose.position.y,
                        geom.pose.position.z);

    state = pdef_->getGoal()->as<ob::GoalState>()->getState();

    // check if any object is close behind the object, which would interfere with the final object push
    bool obj_behind = false;
    for (int i = 0; i < collision_boxes_.size(); i++)
    {
        if (collision_boxes_[i].name == geom.name ||
            collision_boxes_[i].name.find(manipulator_.getName()) != std::string::npos)
        { continue; }
        
        if (std::abs(collision_boxes_[i].pose.position.y - pos.y()) < 0.03 &&
            std::abs(collision_boxes_[i].pose.position.x) - std::abs(pos.x()) <= 0.1 &&
            std::abs(collision_boxes_[i].pose.position.x) > std::abs(pos.x()))
        {
            obj_behind = true;
            break;
        }
    }

    // the final object push is not performed if there is an object behind the goal
    double desired_x;
    if (!obj_behind)
    {
        int direction = (pos.x() < 0) ? -1 : 1;
        double max_x = std::sqrt( (0.94*0.94) - (pos.y()*pos.y()) - (pos.z()*pos.z()) );
        desired_x = util::clamp<double>(pos.x() + direction*0.05, -1*max_x, max_x);
    }
    else
    {
        desired_x = pos.x();
    }

    state->setX(desired_x);
    state->setY(pos.y());
    state->setZ(pos.z());

    state_checker_->setTargetCollision(false);

    bool valid = state_checker_->isValid(state.get());
    if (!valid)
    {
        state->setX(pos.x());

        // recheck state validity
        valid = state_checker_->isValid(state.get());
        if (!valid)
        {
            std::cout << RED << "[PLANNER]: Push grasp action not possible!" << std::endl;
            return false;
        }
    }

    return true;
}

void Planner::executeAction(Planner::ActionType action)
{
    trajectory_msgs::JointTrajectory traj;
    this->generateTrajectory(traj);

    // initial gripper state
    if (action == ActionType::GRASP || action == ActionType::PUSH_GRASP) { controller_.openGripper(); }
    else { controller_.closeGripper(); }

    if (action == ActionType::GRASP)
    {
        int num_states = path_states_ - 1;
        int num_trajectories = traj.points.size()/num_states;

        for (int i = 0; i < num_trajectories; i++)
        {
            trajectory_msgs::JointTrajectory split_traj;
            split_traj.joint_names = traj.joint_names;
            split_traj.points.reserve(num_states);
            split_traj.points.insert(split_traj.points.begin(), traj.points.begin()+num_states*i,traj.points.end()-num_states*(num_trajectories-1-i));
            for (int j = 0; j < split_traj.points.size(); j++)
            {
                split_traj.points[j].time_from_start = ros::Duration(15 * (j+1.0)/split_traj.points.size());
            }
            controller_.sendAction(split_traj);

            // final gripper state
            // i = num_trajectories-2 --> final state in path after which arm should release object
            // i = num_trajectories-3 --> first (or second) state in path after which arm should grasp object
            // the sleep durations are to allow the grasp plugin to attach/detach the target objects
            if (i == num_trajectories-2) { controller_.openGripper(); ros::Duration(1).sleep(); }
            else if (i == num_trajectories-3) { controller_.closeGripper(); ros::Duration(1).sleep(); }
        }
    }
    else
    {
        controller_.sendAction(traj);

        // final gripper state
        // gripper is opened after pushing actions because sometimes object can get stuck inside
        // the gripper during the pushing operation, so this is done to release them
        // consequently, this does mean that they might be dropped off on the ground
        if (action == ActionType::PUSH_GRASP) { controller_.closeGripper(); }
        else { controller_.openGripper(); ros::Duration(1).sleep(); }
    }
}

void Planner::resetArm()
{
    std::vector<Eigen::Vector3d> start_positions;
    std::vector<Eigen::Vector3d> goal_positions;
    std::vector<Eigen::Quaterniond> goal_orientations;
    manipulator_.solveFK(start_positions, goal_orientations);
    manipulator_.solveFK(goal_positions, goal_orientations, manipulator_.getInitPose());

    controller_.openGripper();

    Eigen::Vector3d diff = start_positions.back() - goal_positions.back();
    if (std::abs(diff.x()) < 5e-2 &&
        std::abs(diff.y()) < 5e-2 &&
        std::abs(diff.z()) < 5e-2)
    { return; }
    
    // fake target geometry
    util::CollisionGeometry geom;
    geom.pose.position.x = goal_positions.back().x();
    geom.pose.position.y = goal_positions.back().y();
    geom.pose.position.z = goal_positions.back().z();

    goal_pos_ = goal_positions.back();
    goal_orient_ = goal_orientations.back();

    this->setTargetGeometry(geom);
    this->update();

    state_checker_->setNonStaticCollisions(false);

    if (!state_checker_->isValid(pdef_->getStartState(0)))
    {
        controller_.goToInit();
        return;
    }

    solutions_.clear();
    pdef_->clearSolutionPaths();
    planner_->clear();
    planner_->setProblemDefinition(pdef_);
    if (!planner_->isSetup()) planner_->setup();

    state_checker_->setNonStaticCollisions(false);
    ob::PlannerStatus solved = planner_->solve(timeout_);

    if (solved == ob::PlannerStatus::EXACT_SOLUTION)
    {
        og::PathSimplifierPtr simplifier = og::PathSimplifierPtr(new og::PathSimplifier(si_));
        simplifier->simplifyMax(*pdef_->getSolutionPath()->as<og::PathGeometric>());
        pdef_->getSolutionPath()->as<og::PathGeometric>()->interpolate(10);

        solutions_.push_back(*pdef_->getSolutionPath()->as<og::PathGeometric>());

        trajectory_msgs::JointTrajectory traj;
        if (this->generateTrajectory(traj))
        {
            this->publishMarkers();
            controller_.sendAction(traj);
        }
    }

    this->clearMarkers();
}

bool Planner::startPlanSrvCallback(planner::start_plan::Request& req, planner::start_plan::Response& res)
{
    ROS_INFO("%s[PLANNER]: Received plan request for [%s]", CYAN, req.target.c_str());

    if (req.target.find("_collision") == std::string::npos) req.target += "_collision";

    this->resetArm();
    
    target_geom_.name = req.target;
    target_geom_.dimension.x = target_geom_.dimension.y = target_geom_.dimension.z = -1;
    this->update();

    res.path_found = res.partial_solution = res.grasp_success = false;
    res.plan_time = res.execution_time = res.num_actions = 0.0;

    if (target_geom_.dimension.x == -1)
    {
        ROS_ERROR("[PLANNER]: Unable to find collision geometry for [%s]", req.target.c_str());
        res.message = "Unable to start, target not found!";
        return true;
    }
    else if (std::sqrt((goal_pos_.x()*goal_pos_.x()) + (goal_pos_.y()*goal_pos_.y()) + (goal_pos_.z()*goal_pos_.z())) > 0.95)
    {
        ROS_ERROR("[PLANNER]: Target is out of reach!");
        res.message = "Unable to start, target is out of reach!";
        return true;
    }

    bool success = this->plan();

    std::cout << CYAN << "[PLANNER]: planning time: " << result_.plan_time << " s    "
        << "execution time: " << result_.execution_time << " s" << std::endl;

    res.path_found = result_.path_found;
    res.partial_solution = result_.partial_solution;
    res.grasp_success = result_.grasp_success;
    res.num_actions = result_.num_actions;
    res.plan_time = result_.plan_time;
    res.execution_time = result_.execution_time;

    if (!res.path_found) { res.message = "Unable to find solution!"; }
    else if (res.partial_solution) { res.message = "Only found partial solution!"; }
    else if (res.grasp_success) { res.message = "Solution found and grasp attempt was successful!"; }
    else { res.message = "Solution found but grasp attempt failed!"; }

    return true;
}

bool Planner::resetArmSrvCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    ROS_INFO("%s[PLANNER]: Recieved srv request to reset arm", CYAN);
    this->resetArm();
    return true;
}
