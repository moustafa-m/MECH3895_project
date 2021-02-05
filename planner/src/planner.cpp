#include "planner/planner.h"

Planner::Planner(ros::NodeHandle* nh)
    : nh_(*nh), manipulator_(nh), controller_(nh, &manipulator_)
{
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

    solutions_.clear();
    planner_->clear();
    planner_->setProblemDefinition(pdef_);
    if (!planner_->isSetup()) planner_->setup();

    // initial planning only computes a path without colliding with static objects, non-static objects are not considered
    auto t_start = HighResClk::now();
    ob::PlannerStatus solved = planner_->solve(timeout_);
    auto t_end = HighResClk::now();

    if (solved != ob::PlannerStatus::EXACT_SOLUTION)
    {
        std::cout << RED << "[PLANNER]: No solution found!" << NC << std::endl;
        plan_time_ = chrono::duration_cast<chrono::milliseconds>(t_end-t_start).count()/1000.0;
        return false;
    }

    // now check if the object is blocked by non-static objects, if it is not, then the inital obtained path is used
    // this check is skipped if the arm needs to reset its pose, which is determined by an empty target name
    std::vector<int> blocking_objs;
    if (!this->isObjectBlocked(blocking_objs) && !target_geom_.name.empty())
    {
        std::vector<ob::ScopedState<ob::SE3StateSpace>> states;
        this->planInClutter(blocking_objs, states);

        // disable checks for collision with target and non-static objects if clutter clearing is needed
        state_checker_->setTargetCollision(false);
        state_checker_->setNonStaticCollisions(false);

        pdef_->clearSolutionPaths();

        t_start = HighResClk::now();
        
        for (int i = 0; i < states.size(); i++)
        {
            std::cout << BLUE << "Solving for state: " << i << std::endl;
            
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
            }
            else
            {
                std::cout << RED << "[PLANNER]: No solution found!\nUnable to solve for state : " << i << "\n" << states[i] << NC << std::endl;
                return false;
            }
        }
        
        t_end = HighResClk::now();
    }
    else
    {
        og::PathSimplifierPtr simplifier = og::PathSimplifierPtr(new og::PathSimplifier(si_));
        simplifier->simplifyMax(*pdef_->getSolutionPath()->as<og::PathGeometric>());
        pdef_->getSolutionPath()->as<og::PathGeometric>()->interpolate(path_states_);
        
        solutions_.push_back(*pdef_->getSolutionPath()->as<og::PathGeometric>());
        this->publishMarkers();
    }

    std::cout << GREEN << "[PLANNER]: Solution found!\n";

    int64_t duration = chrono::duration_cast<chrono::milliseconds>(t_end-t_start).count();
    std::cout << CYAN << "Planner took: " << duration << " ms (" << duration/1000.0 << " sec)" << std::endl;
    plan_time_ = duration/1000.0;

    if (save_path_ || display_path_) { this->savePath(); }

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
    pdef_->clearGoal();
    pdef_->clearSolutionPaths();
    pdef_->setGoalState(goal_state);

    goal_pos_ = goal;
    goal_orient_ = orientation;
    this->publishGoalMarker();

    std::cout << BLUE << "[PLANNER]: Goal pose set to:\n {"
        << goal_state->getX() << ", " << goal_state->getY() << ", "
        << goal_state->getZ() << "} {" << goal_state->rotation().x << ", "
        << goal_state->rotation().y << ", " << goal_state->rotation().z << ", "
        << goal_state->rotation().w << "}" << std::endl;
}

void Planner::setCollisionGeometries(const std::vector<util::CollisionGeometry>& collision_boxes)
{
    collision_boxes_.clear();
    collision_boxes_ = collision_boxes;
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

    if (display_path_)
    {
        std::string dir = ros::package::getPath("planner") + "/paths";
        std::string cmd = "cd " + dir + " && ./plot.sh -f " + planner_->getName() + "/" + ss.str() + ".txt";
        system(cmd.c_str());

        if (!save_path_)
        {
            cmd = "rm " + pathtxt + " " + statstxt;
            system(cmd.c_str());
        }
    }
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
    models_sub_ = nh_.subscribe("/gazebo/model_states", 10, &Planner::modelStatesCallback, this);
    start_plan_srv_ = nh_.advertiseService("/start_plan", &Planner::startPlanSrvCallback, this);
    collisions_client_ = nh_.serviceClient<gazebo_geometries_plugin::geometry>("/gazebo/get_geometry", 10);

    ros::param::param<std::vector<std::string>>("/gazebo/static_objects", static_objs_, {"INVALID"});
    ros::param::param<std::string>("~planner/name", planner_name_, "KPIECE1");
    ros::param::param<int>("~planner/timeout", timeout_, 60);
    ros::param::param<int>("~planner/path_states", path_states_, 10);
    ros::param::param<bool>("~planner/save_path", save_path_, false);
    ros::param::param<bool>("~planner/display_path", display_path_, false);

    if (planner_name_ != "KPIECE1" && planner_name_ != "BFMT" && planner_name_ != "RRTStar")
    {
        ROS_WARN("[PLANNER]: unrecognised planner name, setting to KPIECE1!");
        planner_name_ = "KPIECE1";
        ros::param::set("~planner/name", "KPIECE1");
    }
    
    ROS_INFO("%s*******PLANNER PARAMS********", BLUE);
    
    std::stringstream ss;
    for (int i = 0; i < static_objs_.size(); i++) { ss << static_objs_[i] << " "; }
    ROS_INFO_STREAM(BLUE << "Static Objs\t: " << ss.str());
    ROS_INFO_STREAM(BLUE << "name\t\t: " << planner_name_);
    ROS_INFO_STREAM(BLUE << "timeout\t: " << timeout_);
    ROS_INFO_STREAM(BLUE << "path_states\t: " << path_states_);
    ROS_INFO_STREAM(BLUE << "save_path\t: " << std::boolalpha << save_path_);
    ROS_INFO_STREAM(BLUE << "display_path\t: " << std::boolalpha << display_path_);
    
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
        if (i == 0) { success = manipulator_.solveIK(angles, position, orientation, manipulator_.getInitPose()); }
        else { success = manipulator_.solveIK(angles, position, orientation, traj.points[i-1].positions); }

        if (success) { traj.points[i].positions = angles; }
        else
        {
            std::cout << RED << "[PLANNER]: IK solver failed for: \npos: { " << position << " }\nquat: { " << orientation.coeffs() << " }" << std::endl;
            return false;
        }
        
        for (int j = 0; j < num_joints; j++)
        {
            traj.points[i].velocities[j] = 0;
            traj.points[i].accelerations[j] = 0;
        }
        traj.points[i].time_from_start = ros::Duration(2+(i*2));

        if (!ros::ok()) { exit(0); }
    }
    return true;
}

void Planner::publishGoalMarker()
{
    visualization_msgs::Marker marker;
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
    collision_boxes_.clear();

    for (size_t i = 0; i < models_->name.size(); i++)
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

                if (temp.name == target_geom_.name)
                {
                    this->setTargetGeometry(temp);
                }
            }
        }
    }

    int num_joints = manipulator_.getNumJoints();
    // collision geometries for Kinova links and fingers
    for (int i = 0 ; i < num_joints+3; i++)
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
}

bool Planner::isObjectBlocked(std::vector<int>& idxs)
{
    bool clear = true;

    for (size_t i = 0; i < collision_boxes_.size(); i++)
    {
        bool is_static = std::any_of(static_objs_.begin(), static_objs_.end(), [this, i](const std::string& str)
            { return collision_boxes_[i].name.find(str) != std::string::npos; });

        bool out_of_workspace = std::abs(collision_boxes_[i].pose.position.x - goal_pos_.x()) > 0.95 ||
            std::abs(collision_boxes_[i].pose.position.y - goal_pos_.y()) > 0.3 ||
            std::abs(collision_boxes_[i].pose.position.z - goal_pos_.z()) > 0.2;
        
        if (is_static || out_of_workspace ||
            collision_boxes_[i].name.find(manipulator_.getName()) != std::string::npos ||
            collision_boxes_[i].name.find(target_geom_.name) != std::string::npos)
        { continue; }

        if (std::abs(collision_boxes_[i].pose.position.y - goal_pos_.y()) < 0.10)
        {
            if (std::abs(collision_boxes_[i].pose.position.x) - std::abs(goal_pos_.x()) < -0.05) idxs.push_back(i);
            clear = false;
        }
    }

    if (!clear)
    {
        if (idxs.empty()) { std::cout << CYAN << "[PLANNER]: Object is blocked" << NC << std::endl; }
        else { std::cout << CYAN << "[PLANNER]: Path blocked, found " << idxs.size() << " objects to clear" << NC << std::endl; }
    }

    return clear;
}

void Planner::planInClutter(std::vector<int> idxs, std::vector<ob::ScopedState<ob::SE3StateSpace>>& states)
{
    ob::ScopedState<ob::SE3StateSpace> state(space_);
    state = pdef_->getStartState(0);

    if (!idxs.empty())
    {
        std::vector<util::CollisionGeometry> objects;
        std::cout << CYAN << "[PLANNER]: { ";
        for (int i = 0; i < idxs.size(); i++)
        {
            int index = idxs[i];
            std::cout << collision_boxes_[index].name << " ";
            objects.push_back(collision_boxes_[index]);
        }
        std::cout << "}" << std::endl;

        // sort by increasing x values
        std::sort(objects.begin(), objects.end(), [](const util::CollisionGeometry& lhs, const util::CollisionGeometry& rhs)
            { return std::abs(lhs.pose.position.x) < std::abs(rhs.pose.position.x); });
        
        // checks for objects that are close together, these objects can be pushed using one action
        for (int i = 0; i < objects.size(); i++)
        {
            if (objects[i].name.find("_merged") != std::string::npos) continue;
            util::CollisionGeometry orig = objects[i];

            for (int j = 0; j < objects.size(); j++)
            {
                if (orig.name == objects[j].name || objects[j].name.find("_merged") != std::string::npos) continue;

                bool intersect = (orig.min.x < objects[j].max.x) && (orig.max.x > objects[j].min.x);

                if (intersect)
                {
                    objects[j].name += "_merged";
                    objects[j].dimension.y += orig.dimension.y;

                    objects[j].pose.position.x = (orig.pose.position.x + objects[j].pose.position.x)/2;
                    objects[j].pose.position.y = (orig.pose.position.y + objects[j].pose.position.y)/2;
                    
                    std::cout << BLUE << "[PLANNER]: merging " << objects[j].name << " with " << orig.name
                            << "\nNew object dimension: " << objects[j].dimension.y << NC << std::endl;

                    objects.erase(objects.begin() + i); 
                }
            }
        }

        // move end effector to match object's Y and Z positions
        state->setX(objects[0].pose.position.x - (objects[0].pose.position.x < 0 ? -1 : 1)*0.30);
        state->setY(goal_pos_.y());
        state->setZ(goal_pos_.z());
        
        states.push_back(state);

        for (int i = 0; i < objects.size(); i++)
        {
            if (objects[i].name == "SKIP") continue;

            this->getPushAction(states, objects, objects[i]);            
        }
    }
    else
    {
        // if the indices array is empty, objects are surronding the object but may be pushed by a grasp attempt
        state->setX(goal_pos_.x()*0.20);
        state->setY(goal_pos_.y());
        state->setZ(goal_pos_.z());
        states.push_back(state);
    }

    // check if any object is close behind the object, which would interfere with the final object push
    bool obj_behind = false;
    for (int i = 0; i < collision_boxes_.size(); i++)
    {
        if (collision_boxes_[i].name == target_geom_.name) continue;
        
        if (std::abs(collision_boxes_[i].pose.position.y - goal_pos_.y()) < 0.03 &&
            std::abs(collision_boxes_[i].pose.position.x) - std::abs(goal_pos_.x()) <= 0.05)
        {
            obj_behind = true;
            break;
        }
    }

    // the final object push is not performed if there is an object behind the goal
    double desired_x;
    if (!obj_behind)
    {
        int direction = (goal_pos_.x() < 0) ? -1 : 1;
        double max_x = std::sqrt( (0.94*0.94) - (goal_pos_.y()*goal_pos_.y()) - (goal_pos_.z()*goal_pos_.z()) );
        desired_x = util::clamp<double>(goal_pos_.x() + direction*0.05, -1*max_x, max_x);
    }
    else
    {
        desired_x = goal_pos_.x();
    }

    state->setX(desired_x);
    state->setY(goal_pos_.y());
    state->setZ(goal_pos_.z());
    if (!state_checker_->isValid(state.get())) { state->setX(goal_pos_.x()); }
    states.push_back(state);
}

bool Planner::getPushAction(std::vector<ob::ScopedState<ob::SE3StateSpace>>& states, std::vector<util::CollisionGeometry>& objs,
    const util::CollisionGeometry& geom)
{
    double init_dist = (1.5*geom.dimension.y) + 0.05;
    int direction = (geom.pose.position.y > target_geom_.pose.position.y) ? 1 : -1;

    // initially move to be directly beside blocking object (based on direction)
    double desired_y = geom.pose.position.y - direction*init_dist;

    ob::ScopedState<ob::SE3StateSpace> push_state(space_);

    push_state = pdef_->getStartState(0);
    push_state->setX(geom.pose.position.x);
    push_state->setY(desired_y);
    push_state->setZ(goal_pos_.z());

    if (!state_checker_->isValid(push_state.get()))
    {
        direction = -1;
        desired_y = geom.pose.position.y - direction*init_dist;
        push_state->setY(desired_y);

        // if 0 -> push action not possible
        direction = (state_checker_->isValid(push_state.get())) ? -1 : 0;
    }

    if (direction == 0)
    {
        ROS_ERROR("[PLANNER]: Failed to get push action");
        return false;
    }

    states.push_back(push_state);

    for (int i = 0; i < objs.size(); i++)
    {
        util::CollisionGeometry other = objs[i];
        if (other.name == geom.name) continue;

        if (std::abs(other.pose.position.x - push_state->getX()) <= 0.02 &&
            std::abs(other.pose.position.y - push_state->getY()) <= 0.1 &&
            std::abs(other.pose.position.z - push_state->getZ()) <= 0.1)
        {
            std::cout << BLUE << "[PLANNER]: Skipping " << other.name << std::endl;
            objs.erase(objs.begin() + i);
        }
    }

    double clearance = 0.1 - std::abs(geom.pose.position.y - target_geom_.pose.position.y);
    double push_dist = clearance + (init_dist - geom.dimension.y*0.5);

    // push action
    push_state->setY(desired_y + direction*(push_dist));

    // propagate the arm state along the path and check if is valid
    for (int i = 1; i <= 10; i++)
    {
        ob::ScopedState<ob::SE3StateSpace> state(space_);
        state = push_state;
        state->setY(desired_y + direction * (push_dist*i/10));

        if (!state_checker_->isValid(state.get()))
        {
            std::cout << RED << "[PLANNER]: Push action not possible!" << std::endl;
            return false;
        }
    }

    states.push_back(push_state);

    // reset
    direction = (goal_pos_.x() < 0) ? 1 : -1;
    push_state->setX(geom.pose.position.x + (direction*0.1));
    push_state->setY(goal_pos_.y());
    states.push_back(push_state);

    return true;
}

bool Planner::getPushGraspAction(const util::CollisionGeometry& geom, ob::ScopedState<ob::SE3StateSpace>& state)
{
    Eigen::Vector3d pos(geom.pose.position.x,
                        geom.pose.position.y,
                        geom.pose.position.z);

    state = pdef_->getStartState(0);

    // check if any object is close behind the object, which would interfere with the final object push
    bool obj_behind = false;
    for (int i = 0; i < collision_boxes_.size(); i++)
    {
        if (collision_boxes_[i].name == geom.name) continue;
        
        if (std::abs(collision_boxes_[i].pose.position.y - pos.y()) < 0.03 &&
            std::abs(collision_boxes_[i].pose.position.x) - std::abs(pos.x()) <= 0.1)
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
    if (!state_checker_->isValid(state.get())) { state->setX(pos.x()); }

    return true;
}

bool Planner::startPlanSrvCallback(planner::start_plan::Request& req, planner::start_plan::Response& res)
{
    ROS_INFO("%s[PLANNER]: Received plan request for [%s]", CYAN, req.target.c_str());

    if (req.target.find("_collision") == std::string::npos) req.target += "_collision";

    target_geom_.name = req.target;
    target_geom_.dimension.x = target_geom_.dimension.y = target_geom_.dimension.z = -1;
    this->update();

    Eigen::Vector3d goal_xyz = Eigen::Vector3d(target_geom_.pose.position.x,
                                            target_geom_.pose.position.y,
                                            target_geom_.pose.position.z);

    if (target_geom_.dimension.x == -1)
    {
        ROS_ERROR("[PLANNER]: Unable to find collision geometry for [%s]", req.target.c_str());
        res.message = "Unable to start, target not found!";
        return true;
    }
    else if (std::sqrt((goal_xyz.x()*goal_xyz.x()) + (goal_xyz.y()*goal_xyz.y()) + (goal_xyz.z()*goal_xyz.z())) > 0.95)
    {
        ROS_ERROR("[PLANNER]: Target is out of reach!");
        res.message = "Unable to start, target is out of reach!";
        return true;
    }

    controller_.goToInit();
    controller_.openGripper();

    std::vector<Eigen::Vector3d> start_positions; std::vector<Eigen::Quaterniond> start_orientations;
    manipulator_.solveFK(start_positions, start_orientations);

    this->clearMarkers();
    this->setStart(start_positions.back(), start_orientations.back());
    this->setGoal(goal_xyz, start_orientations.back());

    bool success = this->plan();

    if (success)
    {
        res.path_found = success;
        res.plan_time = plan_time_;

        trajectory_msgs::JointTrajectory traj;

        if (!this->generateTrajectory(traj))
        {
            res.path_valid = res.grasp_success = false;
            ROS_ERROR("[PLANNER]: Solution path found but is not kinematically valid!");
            res.message = "Solution path found but is not kinematically valid!";
            return true;
        }

        res.path_valid = true;

        controller_.sendAction(traj);
        controller_.closeGripper();

        // verify grasp attempt
        std::vector<Eigen::Vector3d> positions;
        std::vector<Eigen::Quaterniond> orientations;
        manipulator_.solveFK(positions, orientations);
        this->update();
        if (positions.back()[0] - target_geom_.pose.position.x >= -2e-2 &&
            std::abs(positions.back()[1] - target_geom_.pose.position.y) <= 2e-2)
        {
            ROS_INFO("%s[PLANNER]: Grasp attempt successful!", GREEN);
            res.grasp_success = true;
            res.message = "Solution found and grasp attempt was successful!";
        }
        else
        {
            ROS_ERROR("[PLANNER]: Grasp attempt failed!");
            res.grasp_success = false;
            res.message = "Solution found but grasp attempt failed!";
        }
    }
    else
    {
        res.path_found = res.path_valid = res.grasp_success = false;
        res.plan_time = plan_time_;
        res.message = "Unable to find solution!";

        ROS_ERROR("[PLANNER]: unable to find solution");
    }

    return true;
}
