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

    std::cout << CYAN << "Planner set to: " << planner->getName() << std::endl;
    
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
    start_state->as<ob::SO3StateSpace::StateType>(1)->x = orientation.x();
    start_state->as<ob::SO3StateSpace::StateType>(1)->y = orientation.y();
    start_state->as<ob::SO3StateSpace::StateType>(1)->z = orientation.z();
    start_state->as<ob::SO3StateSpace::StateType>(1)->w = orientation.w();
    pdef_->clearStartStates();
    pdef_->addStartState(start_state);

    std::cout << MAGENTA << "[DEBUG] Start: {" << start_state->getX() << ", " << start_state->getY() << ", " 
            << start_state->getZ() << "} {" << start_state->as<ob::SO3StateSpace::StateType>(1)->x << ", "
            << start_state->as<ob::SO3StateSpace::StateType>(1)->y << ", "
            << start_state->as<ob::SO3StateSpace::StateType>(1)->z << ", "
            << start_state->as<ob::SO3StateSpace::StateType>(1)->w << "}" << std::endl;
}

void Planner::setGoal(const Eigen::Vector3d& goal, const std::string& obj_name)
{
    target_name_ = obj_name;
    ob::ScopedState<ob::SE3StateSpace> goal_state(space_);
    goal_state->setXYZ(goal[0], goal[1], goal[2]);
    //TODO: implement some method to determine desired orientation
    // goal_state->as<ob::SO3StateSpace::StateType>(1)->x = -0.5;
    // goal_state->as<ob::SO3StateSpace::StateType>(1)->y = 0.5;
    // goal_state->as<ob::SO3StateSpace::StateType>(1)->z = 0.5;
    // goal_state->as<ob::SO3StateSpace::StateType>(1)->w = 0.5;
    goal_state->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
    pdef_->clearGoal();
    pdef_->setGoalState(goal_state);

    std::cout << MAGENTA << "[DEBUG] Goal: {" << goal_state->getX() << ", " << goal_state->getY() << ", "
            << goal_state->getZ() << "} {" << goal_state->as<ob::SO3StateSpace::StateType>(1)->x << ", "
            << goal_state->as<ob::SO3StateSpace::StateType>(1)->y << ", "
            << goal_state->as<ob::SO3StateSpace::StateType>(1)->z << ", "
            << goal_state->as<ob::SO3StateSpace::StateType>(1)->w << "}" << std::endl;
}

void Planner::setCollisionGeometries(const std::vector<util::CollisionGeometry>& collision_boxes)
{
    collision_boxes_ = collision_boxes;
}

void Planner::setManipulatorName(const std::string& name)
{
    manipulator_name_ = name;
}

void Planner::savePath(const og::PathGeometric& path)
{
    auto time = std::time(nullptr);
    
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time), "%b-%a-%R");
    std::string s = ros::package::getPath("planner") + "/paths/" + ss.str() + ".txt";
    
    std::cout << GREEN << "Saving solution path to paths/" << ss.str() << ".txt\n" <<
            "Run Python or Bash scripts in paths/ to visualise" << std::endl;
    
    std::ofstream file;
    file.open(s, std::fstream::out);
    path.printAsMatrix(file);
    file.flush();
    file.close();

    std::string dir = ros::package::getPath("planner") + "/paths";
    std::string cmd = "cd " + dir + "&& ./plot.sh -f " + ss.str() + ".txt";
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