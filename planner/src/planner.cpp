#include "planner/planner.h"

Planner::Planner()
{
    this->init();
}

Planner::~Planner()
{

}

og::PathGeometric Planner::plan()
{
    ob::PlannerPtr planner(new og::BFMT(si_));
    planner->setProblemDefinition(pdef_);
    planner->setup();

    name_ = planner->getName();
    std::cout << CYAN << "Planner set to: " << name_ << std::endl;
    
    auto t_start = HighResClk::now();
    ob::PlannerStatus solved = planner->solve(60);
    auto t_end = HighResClk::now();

    int64_t duration = chrono::duration_cast<chrono::milliseconds>(t_end-t_start).count();
    std::cout << CYAN << "Planner took: " << duration << " ms (" << duration/1000.0 << " sec)" << std::endl;

    if (solved == ob::PlannerStatus::APPROXIMATE_SOLUTION)
    {
        ROS_WARN("Approximate solution, attempting to replan with timeout of 30 seconds!");
        t_start = HighResClk::now();
        planner->solve(30);
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
    }
    else
    {
        std::cout << RED << "No solution found!" << NC << std::endl;
        exit(-1);
    }
    
    return *pdef_->getSolutionPath()->as<og::PathGeometric>();
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

void Planner::setGoal(const Eigen::Vector3d& goal, const Eigen::Quaterniond& orientation, const std::string& obj_name)
{
    target_name_ = obj_name;
    ob::ScopedState<ob::SE3StateSpace> goal_state(space_);
    goal_state->setXYZ(goal[0], goal[1], goal[2]);
    goal_state->rotation().x = orientation.x();
    goal_state->rotation().y = orientation.y();
    goal_state->rotation().z = orientation.z();
    goal_state->rotation().w = orientation.w();
    pdef_->clearGoal();
    pdef_->setGoalState(goal_state);

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
    collision_boxes_ = collision_boxes;
}

void Planner::setManipulatorName(const std::string& name)
{
    manipulator_name_ = name;
}

void Planner::savePath()
{
    const og::PathGeometric& path = *pdef_->getSolutionPath()->as<og::PathGeometric>();
    auto time = std::time(nullptr);
    
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time), "%y-%m-%d-%X");
    std::string s = ros::package::getPath("planner") + "/paths/" + name_ + "/";

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
    
    std::cout << GREEN << "Saving solution path to paths/" << name_ << "/" << ss.str() << ".txt\n" <<
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
    std::string cmd = "cd " + dir + "&& ./plot.sh -f " + name_ + "/" + ss.str() + ".txt";
    system(cmd.c_str());
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
    si_->setStateValidityChecker(ob::StateValidityCheckerPtr(new StateChecker(si_, &manipulator_name_, &collision_boxes_)));
    si_->setStateValidityCheckingResolution(0.001);
    si_->setup();

    // problem definition
    pdef_ = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si_));
    pdef_->setOptimizationObjective(ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si_)));
}

// int main(int argc, char** argv)
// {
//     std::cout << "Starting planner test..." << std::endl;

//     Planner planner;
//     planner.plan();

//     std::cout << "Test complete, exiting!" << std::endl;
    
//     return EXIT_SUCCESS;
// }