#include "planner/planner.h"

Planner::Planner()
{
    this->init();
}

Planner::~Planner()
{

}

void Planner::setStart(Eigen::Vector3d& start)
{
    ob::ScopedState<ob::SE3StateSpace> start_state(space_);
    start_state->setXYZ(start[0], start[1], start[2]);
    start_state->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
    pdef_->clearStartStates();
    pdef_->addStartState(start_state);

    std::cout << MAGENTA << "[DEBUG] Start: {" << start_state->getX() << ", " << start_state->getY() << ", " 
            << start_state->getZ() << "}" << std::endl;
}

void Planner::setGoal(Eigen::Vector3d& goal)
{
    ob::ScopedState<ob::SE3StateSpace> goal_state(space_);
    goal_state->setXYZ(goal[0], goal[1], goal[2]);
    goal_state->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
    pdef_->clearGoal();
    pdef_->setGoalState(goal_state);

    std::cout << MAGENTA << "[DEBUG] Goal: {" << goal_state->getX() << ", " << goal_state->getY() << ", "
            << goal_state->getZ() << "}" << std::endl;
}

void Planner::savePath(const og::PathGeometric& path)
{
    auto time = std::time(nullptr);
    
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time), "%b-%a-%R");
    std::string s = ros::package::getPath("planner") + "/paths/" + ss.str() + ".txt";
    
    std::cout << GREEN << "Saving solution path to src/paths/" << ss.str() << ".txt\n" <<
            "Run Python or Bash scripts in paths/ to visualise" << std::endl;
    
    std::ofstream file;
    file.open(s, std::fstream::out);
    path.printAsMatrix(file);
    file.flush();
    file.close();
}

void Planner::init()
{
    space_ = ob::StateSpacePtr(std::make_shared<ob::SE3StateSpace>());

    // TODO: modify for Gazebo env
    ob::RealVectorBounds bounds(3);
    bounds.setHigh(0, 1); bounds.setLow(0, -5); // x dimension
    bounds.setHigh(1, 5); bounds.setLow(1, -5); // y dimension
    bounds.setHigh(2, 1.5); bounds.setLow(2, 0); // z dimension

    space_->as<ob::SE3StateSpace>()->setBounds(bounds);
    
    // space information and validity checker
    si_ = ob::SpaceInformationPtr(new ob::SpaceInformation(space_));
    si_->setStateValidityChecker(ob::StateValidityCheckerPtr(new StateChecker(si_)));
    si_->setup();

    // problem definition
    pdef_ = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si_));
    pdef_->setOptimizationObjective(ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si_)));
}

og::PathGeometric Planner::plan()
{
    // og::SimpleSetup ss(space_);

    // ss.setStateValidityChecker([this](const ob::State* state) { return isStateValid(state); });

    // ob::ScopedState<ob::SE3StateSpace> start(space_);
    // start.random();

    // ob::ScopedState<ob::SE3StateSpace> goal(space_);
    // goal.random();

    // pdef_->setStartAndGoalStates(start, goal);
    

    ob::PlannerPtr planner(new og::RRTstar(si_));
    planner->setProblemDefinition(pdef_);
    planner->setup();

    std::cout << CYAN << "Using " << planner->getName() << std::endl;

    // ss.setStartAndGoalStates(start, goal);
    
    ob::PlannerStatus solved = planner->solve(1.0);

    if (solved)
    {
        std::cout << GREEN << "Solution found!\n";

        #ifdef DEBUG
        std::cout << MAGENTA << "-----\nDEBUG\n-----\nObtained path:\n";
        pdef_->getSolutionPath()->print(std::cout);
        std::cout << "With cost: " << pdef_->getSolutionPath()->cost(pdef_->getOptimizationObjective())
                << " and length: " << pdef_->getSolutionPath()->length() << std::endl;

        std::cout << "Path in Matrix form: \n";
        std::static_pointer_cast<og::PathGeometric>(pdef_->getSolutionPath())->printAsMatrix(std::cout);
        std::cout << "-----" << NC << std::endl;
        #endif
        
        this->savePath(*pdef_->getSolutionPath()->as<og::PathGeometric>());
    }
    else
    {
        std::cout << RED << "No solution found!" << NC << std::endl;
        exit(-1);
    }
    
    return *pdef_->getSolutionPath()->as<og::PathGeometric>();
}

// int main(int argc, char** argv)
// {
//     std::cout << "Starting planner test..." << std::endl;

//     Planner planner;
//     planner.plan();

//     std::cout << "Test complete, exiting!" << std::endl;
    
//     return EXIT_SUCCESS;
// }