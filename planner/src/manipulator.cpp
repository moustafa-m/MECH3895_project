#include "planner/manipulator.h"

Manipulator::Manipulator(ros::NodeHandle* nh)
    : nh_(*nh)
{
    ros::param::param<std::string>("/robot_name", name_, "j2s7s300");
    
    ROS_INFO("%sRobot name set to %s", BLUE, name_);
    
    this->setJointsInfo();
    this->initSolvers();
    this->setDefaultPoses();
    this->setLimits();
    this->setDHParameters();

    joints_sub_ = nh_.subscribe("/" + name_ +"/joint_states", 10, &Manipulator::jointStatesCallback, this);
}

Manipulator::~Manipulator()
{
    delete ik_solver_;
    delete fk_solver_;
}

std::vector<double> Manipulator::solveIK(Eigen::Vector3d position)
{
    KDL::JntArray current_states(num_joints_), out(num_joints_);
    std::cout << GREEN << "Current angles: {";
    for (int i = 0; i < num_joints_; i++) { current_states(i) = joint_states_.position[i]; std::cout << joint_states_.position[i] << " ";}
    std::cout << "}" << NC << std::endl;


    KDL::Frame end_effector_pose;
    fk_solver_->JntToCart(current_states, end_effector_pose);
    
    double r, p, y;
    end_effector_pose.M.GetRPY(r, p, y);

    #ifdef DEBUG
    std::cout << MAGENTA << "-----\nDEBUG\n-----\nEnd effector current Pose: \nPos: {" << end_effector_pose.p[0]
            << ", " << end_effector_pose.p[1] << ", " << end_effector_pose.p[2] <<
            "}\n" << "RPY: {" << r << ", " << p << ", " << y << "}\n-----" << NC << std::endl;
    #endif

    // end_effector_pose.M = KDL::Rotation::RPY(-M_PI_2, 0, M_PI_2);
    end_effector_pose.M.GetRPY(r, p, y);
    end_effector_pose.p[0] = position[0]; end_effector_pose.p[1] = position[1]; end_effector_pose.p[2] = position[2];
    
    #ifdef DEBUG
    std::cout << MAGENTA << "DEBUG\n-----\nEnd Effector desired Pose: \n" << "Pos: {" << end_effector_pose.p[0]
            << ", " << end_effector_pose.p[1] << ", " << end_effector_pose.p[2] << "}\nRPY: {" << r << ", "
            << p << ", " << y << "}\n-----" << NC << std::endl;
    #endif

    int success = ik_solver_->CartToJnt(current_states, end_effector_pose, out);

    if (success < 0)
    {
        ROS_ERROR_STREAM("Failed to obtain IK solution!");
        exit(-1);
    }

    ROS_INFO("%sIK solution found!", GREEN);
    
    std::vector<double> output(num_joints_);
    std::cout << GREEN << "Obtained angles: {";
    for (int i = 0; i < num_joints_; i++) {
        output[i] = out(i);
        std::cout << output[i] << " ";
    }
    std::cout << "}" << NC << std::endl;

    return output;
}

std::vector<Eigen::Vector3d> Manipulator::solveFK()
{
    // std::vector<Eigen::Matrix4d> matrices(num_joints_);

    // this->setDHTheta();
    // for (int i = 0; i < num_joints_; i++)
    // {
    //     matrices[i] << cos(dh_para_.theta[i]), -sin(dh_para_.theta[i])*cos(dh_para_.alpha[i]), sin(dh_para_.theta[i])*sin(dh_para_.alpha[i]), dh_para_.a_i[i]*cos(dh_para_.theta[i]),
    //             sin(dh_para_.theta[i]), cos(dh_para_.theta[i])*cos(dh_para_.alpha[i]), -cos(dh_para_.theta[i])*sin(dh_para_.alpha[i]), dh_para_.a_i[i]*sin(dh_para_.theta[i]),
    //             0, sin(dh_para_.alpha[i]), cos(dh_para_.alpha[i]), dh_para_.d_i[i],
    //             0, 0, 0, 1;
    // }

    // Eigen::Matrix4d T0_N = matrices[0];
    // for (int i = 1; i < matrices.size(); i++) { T0_N *= matrices[i]; }
    // #ifdef DEBUG
    // std::cout << "End effector Matrix: \n" << T0_N << NC << std::endl;
    // #endif

    KDL::JntArray current_states(num_joints_);
    for (int i = 0; i < num_joints_; i++) { current_states(i) = joint_states_.position[i]; }

    KDL::Frame end_effector_pose;
    fk_solver_->JntToCart(current_states, end_effector_pose);
    Eigen::Vector3d position, kinova_orientation;
    position[0] = end_effector_pose.p[0]; 
    position[1] = end_effector_pose.p[1];
    position[2] = end_effector_pose.p[2];

    end_effector_pose.M.GetRPY(kinova_orientation[0], kinova_orientation[1], kinova_orientation[2]);
    // kinova_orientation[0] = -asin(T0_N(2,0));
    // kinova_orientation[1] = atan2(T0_N(2,1)/cos(kinova_orientation[0]), T0_N(2,2)/cos(kinova_orientation[0]));
    // kinova_orientation[2] = atan2(T0_N(1,0)/cos(kinova_orientation[0]), T0_N(0,0)/cos(kinova_orientation[0]));

    // position = transform_*position;
    // kinova_orientation = transform_*kinova_orientation;

    #ifdef DEBUG
    std::cout << MAGENTA << "DEBUG\n------\nend effector position:\n" << position << "\nend effector orientation:\n"
            << kinova_orientation << "\n------" << NC << std::endl;
    #endif

    std::vector<Eigen::Vector3d> pose = {position, kinova_orientation};
    return pose;
}

std::string Manipulator::getName()
{
    return name_;
}

int Manipulator::getNumJoints()
{
    return num_joints_;
}

std::vector<std::string> Manipulator::getJointNames()
{
    return joint_names_;
}

std::vector<std::string> Manipulator::getFingerNames()
{
    return finger_names_;
}

std::vector<double> Manipulator::getHomePose()
{
    return home_pose_;
}

std::vector<double> Manipulator::getInitPose()
{
    return init_pose_;
}

void Manipulator::setJointsInfo()
{
    if (name_.compare("j2n6s300") == 0 || name_.compare("j2s6s300") == 0) { num_joints_ = 6; }
    else if (name_.compare("j2s7s300") == 0) { num_joints_ = 7; }
    else { ROS_ERROR_STREAM("Invalid Robot name! input: " << name_); exit(-1); }

    joint_names_.resize(num_joints_); finger_names_.resize(3);
    for (int i = 0; i < num_joints_; i++)
    {
        joint_names_[i] = name_ + "_joint_" + std::to_string(i+1);
        if (i < 3) { finger_names_[i] = name_ + "_joint_finger_" + std::to_string(i+1); }
    }
}

void Manipulator::initSolvers()
{
    std::string chain_start, chain_end;
    chain_start = name_ + "_link_base";
    chain_end = name_ + "_end_effector";
    ik_solver_ = new TRAC_IK::TRAC_IK(chain_start, chain_end, "/robot_description", 10, 1e-3, TRAC_IK::Distance);

    bool valid = ik_solver_->getKDLChain(chain_);
    if (!valid)
    {
        ROS_ERROR("Invalid kinematic chain!");
        exit(-1);
    }

    valid = ik_solver_->getKDLLimits(kdl_lower_b_, kdl_upper_b_);
    if (!valid)
    {
        ROS_ERROR("Invalid upper and lower bounds!");
        exit(-1);
    }

    // kdl_lower_b_.data[4] = -M_PI; kdl_lower_b_.data[5] = -M_PI; kdl_lower_b_.data[6] = -M_PI;
    // kdl_upper_b_.data[4] = M_PI; kdl_upper_b_.data[5] = M_PI; kdl_upper_b_.data[6] = M_PI;
    // printing upper and lower bounds - for sanity checks if needed
    // std::cout << "{ ";
    // for (int i = 0; i < num_joints_; i++)
    // {
    //     std::cout << "(" << kdl_lower_b_(i) << ", " << kdl_upper_b_(i) << ") ";
    // }
    // std::cout << " }" << std::endl;

    assert(chain_.getNrOfJoints() == num_joints_);
    assert(chain_.getNrOfJoints() == kdl_lower_b_.data.size());
    assert(chain_.getNrOfJoints() == kdl_upper_b_.data.size());

    fk_solver_ = new KDL::ChainFkSolverPos_recursive(chain_);
}

void Manipulator::setDefaultPoses()
{
    home_pose_.resize(num_joints_); init_pose_.resize(num_joints_);

    if (name_.compare("j2n6s300") == 0 || name_.compare("j2s6s300") == 0)
    {
        home_pose_ = {0.0, 2.9, 1.3, 4.2, 1.4, 0.0};

        // check if curved or spherical wrist
        if (name_.compare("j2n6s300") == 0)
        {
            init_pose_ = {0.0, M_PI, 0.6, 2.4, 1.4, 0}; // for j2n6s300 (curved wrist)
        }
        else
        {
            // init_pose_ = {M_PI, 3*M_PI/2, M_PI_2, 3*M_PI/2, 3*M_PI/2, 3*M_PI/2}; // for j2s6s300 (spherical wrist)
            init_pose_ = {0.0, M_PI, 1.3, 0.0, 2.9, 0.0}; // for j2s6s300 (spherical wrist)
        }
    }
    else
    {
        home_pose_ = {0.0, 2.9, 0.0, 1.3, 4.2, 1.4, 0.0};
        init_pose_ = {0.0, 3*M_PI_4, 0.0, 0.8, -M_PI, M_PI, M_PI};
    }
}

void Manipulator::setLimits()
{
    upper_bounds_.resize(num_joints_); lower_bounds_.resize(num_joints_);

    for (int i = 0; i < num_joints_; i++)
    {
        upper_bounds_[i] = kdl_upper_b_.data[i];
        lower_bounds_[i] = kdl_lower_b_.data[i];
    }
}

void Manipulator::setDHParameters()
{
    dh_para_.alpha.resize(num_joints_);
    dh_para_.a_i.resize(num_joints_);
    dh_para_.d_i.resize(num_joints_);

    if (name_.compare("j2n6s300") == 0)
    {
        dh_para_.alpha = {M_PI_2, M_PI, M_PI_2, M_PI/3, M_PI/3, M_PI};
        dh_para_.a_i = {0, 0.41, 0, 0, 0, 0};
        dh_para_.d_i = {0.2755, 0, -0.0098, -0.2672, -0.1199, -0.1199};

        transform_ << 1, 0, 0,
                    0, 1, 0,
                    0, 0, 1;
    }
    else if (name_.compare("j2s6s300") == 0)
    {
        dh_para_.alpha = {M_PI_2, M_PI, M_PI_2, M_PI_2, M_PI_2, M_PI};
        dh_para_.a_i = {0, 0.41, 0, 0, 0, 0};
        dh_para_.d_i = {0.2755, 0, -0.0098, -0.3111, 0, -0.2638};
    }
    else
    {
        dh_para_.alpha = {M_PI_2, M_PI_2, M_PI_2, M_PI_2, M_PI_2, M_PI_2, M_PI};
        dh_para_.a_i = {0, 0, 0, 0, 0, 0, 0};
        dh_para_.d_i = {-0.2755, 0, -0.41, -0.0098, -0.3111, 0, -0.2638};
    }

    transform_ << 1, 0, 0,
                0, -1, 0,
                0, 0, -1;
}

void Manipulator::setDHTheta()
{
    dh_para_.theta.resize(num_joints_);

    if (name_.compare("j2n6s300") == 0)
    {
        dh_para_.theta[0] = joint_states_.position[0]*-1;
        dh_para_.theta[1] = joint_states_.position[1] - M_PI_2;
        dh_para_.theta[2] = joint_states_.position[2] + M_PI_2;
        dh_para_.theta[3] = joint_states_.position[3];
        dh_para_.theta[4] = joint_states_.position[4] - M_PI;
        dh_para_.theta[5] = joint_states_.position[5] + M_PI_2;
    }
    else if (name_.compare("j2s6s300") == 0)
    {
        dh_para_.theta[0] = joint_states_.position[0] - M_PI;
        dh_para_.theta[1] = joint_states_.position[1] + M_PI_2;
        dh_para_.theta[2] = joint_states_.position[2] + M_PI_2;
        dh_para_.theta[3] = joint_states_.position[3];
        dh_para_.theta[4] = joint_states_.position[4];
        dh_para_.theta[5] = joint_states_.position[5] - M_PI_2;
    }
    else
    {
        dh_para_.theta[0] = joint_states_.position[0] - M_PI;
        dh_para_.theta[1] = joint_states_.position[1];
        dh_para_.theta[2] = joint_states_.position[2];
        dh_para_.theta[3] = joint_states_.position[3];
        dh_para_.theta[4] = joint_states_.position[4];
        dh_para_.theta[5] = joint_states_.position[5] - M_PI_2;
        dh_para_.theta[6] = joint_states_.position[6];          //TODO: NEEDS VERIFICATION!!!!
    }
}

void Manipulator::jointStatesCallback(sensor_msgs::JointStateConstPtr msg)
{
    joint_states_ = *msg;
}