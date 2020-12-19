#include "planner/manipulator.h"

Manipulator::Manipulator(ros::NodeHandle* nh)
    : nh_(*nh)
{
    ros::param::param<std::string>("/robot_name", name_, "j2s7s300");
    
    ROS_INFO("%sRobot name set to %s", BLUE, name_.c_str());
    
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

std::vector<double> Manipulator::solveIK(Eigen::Vector3d position, Eigen::Quaterniond orientation, std::vector<double> prev_joints)
{
    #ifdef DEBUG
    std::cout << MAGENTA << "-----\n[DEBUG]\nCurrent angles: {";
    for (int i = 0; i < num_joints_; i++) { std::cout << joint_states_.position[i] << " "; }
    std::cout << "}" << std::endl;

    std::cout << MAGENTA << "Prev angles: {";
    for (int i = 0; i < num_joints_; i++) { std::cout << prev_joints[i] << " "; }
    std::cout << "}" << NC << std::endl;
    #endif

    KDL::JntArray current_states(num_joints_), out(num_joints_);
    for (int i = 0; i < num_joints_; i++) { current_states(i) = prev_joints[i]; }

    #ifdef DEBUG
    double r, p, y;
    
    std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Quaterniond>> prev_pose;
    prev_pose = this->solveFK(prev_joints);
    KDL::Rotation m = KDL::Rotation::Quaternion(prev_pose.second.back().x(), prev_pose.second.back().y(), prev_pose.second.back().z(),
                prev_pose.second.back().w());
    m.GetRPY(r, p, y);

    std::cout << MAGENTA << "-----\n[DEBUG]\n-----\nEnd effector prev Pose: \nPos: {" << prev_pose.first.back().x()
            << ", " << prev_pose.first.back().y() << ", " << prev_pose.first.back().z() <<
            "}\nRPY: {" << r << ", " << p << ", " << y << "}\n-----" << NC << std::endl;
    #endif

    KDL::Frame end_effector_pose;
    end_effector_pose.M = KDL::Rotation::Quaternion(orientation.x(), orientation.y(), orientation.z(), orientation.w());
    end_effector_pose.p[0] = position[0]; end_effector_pose.p[1] = position[1]; end_effector_pose.p[2] = position[2];
    
    #ifdef DEBUG
    end_effector_pose.M.GetRPY(r, p, y);
    std::cout << MAGENTA << "[DEBUG]\n-----\nEnd Effector desired Pose: \n" << "Pos: {" << end_effector_pose.p[0]
            << ", " << end_effector_pose.p[1] << ", " << end_effector_pose.p[2] << "}\nRPY: {" << r << ", "
            << p << ", " << y << "}\n-----" << NC << std::endl;
    #endif

    int success = ik_solver_->CartToJnt(current_states, end_effector_pose, out);

    if (success < 0)
    {
        ROS_ERROR_STREAM("Failed to obtain IK solution!\nFailed for state:\n"<< "Pos: {" << end_effector_pose.p[0]
            << ", " << position[0] << ", " << position[1] << "}\nQuaternion: {" << position[2] << ", "
            << orientation.y() << ", " << orientation.z() << ", " << orientation.w() << "}\n-----" << NC << std::endl);
    }

    std::vector<double> output(num_joints_);
    for (int i = 0; i < num_joints_; i++) { output[i] = out(i); }

    return output;
}

std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Quaterniond>> Manipulator::solveFK(const std::vector<double>& joints_pos)
{
    KDL::JntArray current_states(num_joints_);
    if (joints_pos.empty())
    {
        for (int i = 0; i < num_joints_; i++) { current_states(i) = joint_states_.position[i]; }
    }
    else
    {
        assert(joints_pos.size() == num_joints_ && "Number of joints in joint_states must be equal to number of joints in the manipulator!");
        for (int i = 0; i < num_joints_; i++) { current_states(i) = joints_pos[i]; }
    }

    std::vector<KDL::Frame> poses;
    poses.resize(chain_.getNrOfSegments());
    KDL::Frame end_effector_pose;
    fk_solver_->JntToCart(current_states, poses);

    std::vector<Eigen::Vector3d> positions; std::vector<Eigen::Quaterniond> quaternions;
    positions.resize(num_joints_); quaternions.resize(num_joints_);
    for (int i = 0; i < positions.size(); i++)
    {
        // FK solver gives root pose at i = 0 which is not needed, so use i+1
        positions[i] << poses[i+1].p[0],
                poses[i+1].p[1],
                poses[i+1].p[2];
        poses[i+1].M.GetQuaternion(quaternions[i].x(), quaternions[i].y(), quaternions[i].z(), quaternions[i].w());
    }

    #ifdef DEBUG
    std::cout << MAGENTA << "-----\n[DEBUG]\n-----\nend effector position:\n" << positions.back() << "\nend effector orientation:\n"
            << quaternions.back().coeffs() << "\n------" << NC << std::endl;
    #endif
    return std::make_pair(positions, quaternions);
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
    ik_solver_ = new TRAC_IK::TRAC_IK(chain_start, chain_end, "/robot_description", 0.5, 1e-3, TRAC_IK::Distance);

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
            init_pose_ = {0.0, 3*M_PI_4, 0.6, 2.4, 1.4, 0}; // for j2n6s300 (curved wrist)
        }
        else
        {
            // init_pose_ = {M_PI, 3*M_PI/2, M_PI_2, 3*M_PI/2, 3*M_PI/2, 3*M_PI/2}; // for j2s6s300 (spherical wrist)
            init_pose_ = {0.0, 3*M_PI_4, 0.8, 0.0, M_PI, 0.0}; // for j2s6s300 (spherical wrist)
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