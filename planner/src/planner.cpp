#include "planner/planner.h"

Planner::Planner()
{
    this->init();
}

Planner::~Planner()
{

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
    // si_->setStateValidityChecker(ob::StateValidityCheckerPtr(new StateChecker(si_)));
    si_->setStateValidityChecker(std::bind(&Planner::isStateValid, this, std::placeholders::_1));
    si_->setup();

    // problem definition
    pdef_ = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si_));
    pdef_->setOptimizationObjective(ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si_)));
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
    std::cout << CYAN << "Planner took: " << duration << " ms " << "(" << duration/1000.0 << " sec)" << std::endl;

    if (solved == ob::PlannerStatus::APPROXIMATE_SOLUTION)
    {
        ROS_WARN("Approximate solution, attempting to replan with timeout of 30 seconds!");
        t_start = HighResClk::now();
        planner->solve(30);
        t_end = HighResClk::now();

        duration = chrono::duration_cast<chrono::milliseconds>(t_end-t_start).count();
        std::cout << CYAN << "Replanning took: " << duration << " ms " << "(" << duration/1000.0 << " sec)" << std::endl;
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

bool Planner::isStateValid(const ob::State* state)
{
    const ob::SE3StateSpace::StateType* state3D = state->as<ob::SE3StateSpace::StateType>();

    double x = state3D->getX(); double y = state3D->getY(); double z = state3D->getZ();
    if (std::sqrt((x*x)+(y*y)+(z*z)) >= 0.985)
    {
        #ifdef DEBUG
        std::cout << MAGENTA << "-----\n[DEBUG]\n" << "State {" << x << ", " << y << ", " << z
                << "} out of reach: " << std::sqrt((x*x)+(y*y)+(z*z)) << "m\n-----" << std::endl;
        #endif
        return false;
    }

    int idx = 0;
    for (size_t i = 0; i < collision_boxes_.size(); i++)
    {
        if (collision_boxes_[i].name.find("_link_7") != std::string::npos)
        {
            idx = i;
            break;
        }
        else if (collision_boxes_[i].name.find("_link_6") != std::string::npos)
        {
            idx = i;
        }
    }

    auto sign = [](double val) { return val < 0 ? -1 : 1; };
    
    //TODO: for now use the last link connected to the end effector to check for collisions
    // IK should be added in state validity to obtain positions of end effector and links
    // to use for collision checking
    util::CollisionGeometry effector = collision_boxes_[idx];
    std::shared_ptr<fcl::CollisionGeometry> effectorbox;
    // create box with enlarged collision box, size is not taken to be exact due to empty space in between fingers
    effectorbox = std::shared_ptr<fcl::CollisionGeometry>(new fcl::Box(effector.dimension.x+0.05, effector.dimension.y+0.02, effector.dimension.z));
    fcl::CollisionObject effectorobj(effectorbox);
    fcl::Vec3f effector_xyz(x-(sign(x)*(0.12+(effector.dimension.x/2))), y, z); // change center of collision box to be at centre of last link
    fcl::Quaternion3f effector_quat(state3D->as<ob::SO3StateSpace::StateType>(1)->w,
                    state3D->as<ob::SO3StateSpace::StateType>(1)->x,
                    state3D->as<ob::SO3StateSpace::StateType>(1)->y,
                    state3D->as<ob::SO3StateSpace::StateType>(1)->z);
    effectorobj.setTransform(effector_quat, effector_xyz);

    // Thumb
    util::CollisionGeometry kinova_thumb = collision_boxes_[collision_boxes_.size()-3];
    std::shared_ptr<fcl::CollisionGeometry> thumbbox;
    thumbbox = std::shared_ptr<fcl::CollisionGeometry>(new fcl::Box(kinova_thumb.dimension.x, kinova_thumb.dimension.y, kinova_thumb.dimension.z));
    fcl::CollisionObject thumbobj(thumbbox);
    fcl::Vec3f thumb_xyz(x-(sign(x)*0.05), y+0.06, z); // change center of thumb
    fcl::Quaternion3f thumb_quat(0.9659258, 0, 0, -0.258819);
    thumbobj.setTransform(thumb_quat, thumb_xyz);

    // fingers NOTE: the collision geometry for the two fingers is taken to be one box for simplification hence z dimension is multiplied by 2 and has 2cm added
    util::CollisionGeometry kinova_finger = collision_boxes_[collision_boxes_.size()-2];
    std::shared_ptr<fcl::CollisionGeometry> fingerbox;
    fingerbox = std::shared_ptr<fcl::CollisionGeometry>(new fcl::Box(kinova_finger.dimension.x, kinova_finger.dimension.y, kinova_finger.dimension.z*2+0.02));
    fcl::CollisionObject fingerobj(fingerbox);
    fcl::Vec3f finger_xyz(x-(sign(x)*0.05), y-0.06, z); // change center of fingers
    fcl::Quaternion3f finger_quat(0.9659258, 0, 0, 0.258819);
    fingerobj.setTransform(finger_quat, finger_xyz);

    for (size_t i = 0; i < collision_boxes_.size(); i++)
    {
        // not checking for self collisions or collisions of other joints with environment
        if (collision_boxes_[i].name.find(manipulator_name_) != std::string::npos) { continue; }

        std::shared_ptr<fcl::CollisionGeometry> box;
        box = std::shared_ptr<fcl::CollisionGeometry>(new fcl::Box(collision_boxes_[i].dimension.x, collision_boxes_[i].dimension.y, collision_boxes_[i].dimension.z));

        fcl::CollisionObject obj(box);
        fcl::Vec3f xyz(collision_boxes_[i].centre.x, collision_boxes_[i].centre.y, collision_boxes_[i].centre.z);
        fcl::Quaternion3f quat(1, 0, 0, 0);
        obj.setTransform(quat, xyz);

        fcl::CollisionRequest request(1, false, 1, false);
        fcl::CollisionResult result, thumb_result, finger_result;
        fcl::collide(&effectorobj, &obj, request, result);
        fcl::collide(&thumbobj, &obj, request, thumb_result);
        fcl::collide(&fingerobj, &obj, request, finger_result);

        if (result.isCollision() || thumb_result.isCollision() || finger_result.isCollision())
        {
            #ifdef DEBUG
            fcl::Vec3f translation; std::string name; geometry_msgs::Vector3 dim; fcl::Contact contact;

            std::cout << MAGENTA << "-----\n[DEBUG]\nCollision with: " << collision_boxes_[i].name << " {"
                    << collision_boxes_[i].centre.x << ", " << collision_boxes_[i].centre.y << ", "
                    << collision_boxes_[i].centre.z << "}" << std::endl;
            std::cout << "Object dimensions: {" << collision_boxes_[i].dimension.x << " " << collision_boxes_[i].dimension.y << " " 
                    << collision_boxes_[i].dimension.z << "}" << std::endl;
            std::cout << "Detected collision in state: {" << x << ", " << y << ", " << z << "}" << std::endl;
            
            if (result.isCollision()) { translation = effectorobj.getTranslation(); name = "effector"; dim = effector.dimension; }
            else if (thumb_result.isCollision()) { translation = thumbobj.getTranslation(); name = "thumb"; dim = kinova_thumb.dimension; }
            else { translation = fingerobj.getTranslation(); name = "finger"; dim = kinova_finger.dimension; }

            std::cout << "link name: " << name << "\nlink position: " << translation << "\nlink dimensions: {"
                    << dim.x << ", " << dim.y << ", " << dim.z << "}\n-----" << std::endl;
            
            #endif
            return false;
        }
    }
    
    return true;
}

// int main(int argc, char** argv)
// {
//     std::cout << "Starting planner test..." << std::endl;

//     Planner planner;
//     planner.plan();

//     std::cout << "Test complete, exiting!" << std::endl;
    
//     return EXIT_SUCCESS;
// }