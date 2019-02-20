#include "lwr_traj_generator/LwrTrajGenerator.h"


LwrTrajGenerator::LwrTrajGenerator(){}

void LwrTrajGenerator::init(ros::NodeHandle &nh){
    nh_=nh;
    flag_work_=false;
    joint_msr_pos_.resize(7);
    home_pos_={0.00, -1.57, 0.00, 0.00, 0.00, 0.00, 0.00};
    cmd_msg_ptr_->data = home_pos_;

    all_traj.push_back({0.00, -1.57, 0.00, 0.00, 0.00, 0.00, 0.00});
    all_traj.push_back({0.00, 1.57, 0.00, 0.00, 0.00, 0.00, 0.00});
    all_traj.push_back({-0.50, 1.00, 0.00, 0.00, 0.00, 0.00, 0.00});
    all_traj.push_back({0.20, 1.00, 1.00, 0.50, 1.00, 0.50, 1.00});
    all_traj.push_back({0.30, 1.00, 1.00, 0.50, 1.00, 0.50, 1.00});
    all_traj.push_back({-0.25, 1.00, 1.00, 0.50, 1.00, 0.50, 1.00});
    all_traj.push_back({-0.00, 0.17, 1.00, 1.00, -1.00, -1.00, -1.00});
    all_traj.push_back({-0.30, 1.17, 1.00, 1.00, -1.00, -1.00, -0.05});
    all_traj.push_back({-0.20, -1.20, 1.00, 1.00, -0.80, -1.10, -0.08});
    all_traj.push_back({0.00, -1.20, 0.00, -1.00, -1.10, 0.00, -1.00});
    all_traj.push_back({-0.80, -0.50, -0.50, -1.00, -1.00, 0.00, -1.00});
    all_traj.push_back({-0.20, 0.17, 1.00, 1.00, -1.00, -1.00, -1.00});
    all_traj.push_back({-0.80, -0.50, -0.50, -1.00, -1.00, 0.00, -1.00});
    all_traj.push_back({-1.50, -0.00, 1.10, -0.60, 0.50, 0.60, -0.02});
    all_traj.push_back({-0.70, 1.00, 0.20, -0.60, 0.50, 0.60, -0.02});
    all_traj.push_back({0.00, -0.70, 0.30, 0.70, 0.20, 1.00, -1.02});
    all_traj.push_back({-0.2, -0.70, 0.50, -0.60, 0.20, 1.00, -1.02});

    pub_posture_ =  nh_.advertise<std_msgs::Float64MultiArray>("/lwr/adaptive_torque_controller/command", 10);
    sub_command_= nh_.subscribe("/lwr/trajectory_generator_command", 1, &LwrTrajGenerator::command, this);
    sub_joint_state_= nh_.subscribe("/lwr/joint_states", 1, &LwrTrajGenerator::getJointPosition, this);

    ROS_INFO ("LwrTrajGenerator is initialized");
}

void LwrTrajGenerator::update(const ros::Time& time, const ros::Duration& period){


  double norm= 0.0;
  for (size_t i = 0; i< joint_msr_pos_.size(); ++i)
    norm += pow(cmd_msg_ptr_->data[i] - joint_msr_pos_[i],2);

    if (flag_work_ && norm < 0.002 || elapsed_time_.toSec()>45.0){
        int rand_traj_num = std::rand() % all_traj.size();
        cmd_msg_ptr_->data = all_traj[rand_traj_num];
        pub_posture_.publish (cmd_msg_ptr_);
        std::cout<< "Measured position: " ; for(const double &p: joint_msr_pos_){std::cout <<p<<" ";} std::cout << "\n";
        std::cout<< "Desired position: " ; for(const double &p: cmd_msg_ptr_->data){std::cout <<p<<" ";} std::cout << "\n";
        elapsed_time_.fromSec(0.0);
    }
    elapsed_time_ = elapsed_time_+period;
}

void LwrTrajGenerator::command(const std_msgs::String &msg){
    std::string cmd = msg.data.c_str();
    if (cmd.compare("start") == 0) {
        ROS_INFO ("Starting...");
        startTrajectroies();
    }

    else if (cmd.compare("stop") == 0) {
        ROS_INFO ("Stoping...");
        stopTrajectroies();
    }

    else if (cmd.compare("home up") == 0) {
        ROS_INFO ("Homing to upper position...");
        goToPosition(home_pos_);
    }

    else if (cmd.compare("home down") == 0) {
        ROS_INFO ("Homing to lower position...");
        goToPosition({0.00, 1.57, 0.00, 0.00, 0.00, 0.00, 0.00});
    }

    else{
        ROS_WARN("Wrong command! The choice is: start, stop, home");
    }

}

void LwrTrajGenerator::startTrajectroies(){
    cmd_msg_ptr_->data = all_traj[std::rand() % all_traj.size()];
    pub_posture_.publish (cmd_msg_ptr_);
    std::cout<< "Desired initial position: " ; for(const double &p: cmd_msg_ptr_->data){std::cout <<p<<" ";} std::cout << "\n";
        elapsed_time_.fromSec(0.0);
    flag_work_ = true;
}

void LwrTrajGenerator::stopTrajectroies(){
    cmd_msg_ptr_->data = joint_msr_pos_;
    pub_posture_.publish (cmd_msg_ptr_);
    std::cout<< "Stop in desired position: " ; for(const double &p: cmd_msg_ptr_->data){std::cout <<p<<" ";} std::cout << "\n";
    flag_work_ = false;
}

void LwrTrajGenerator::goToPosition(const std::vector<double> &goal_pos){
    if(goal_pos.size() != cmd_msg_ptr_->data.size())
        ROS_WARN ("Incorrect size for goal position");
    else{
        cmd_msg_ptr_->data = goal_pos;
        pub_posture_.publish (cmd_msg_ptr_);
        std::cout<< "Desired position: " ; for(const double &p: cmd_msg_ptr_->data){std::cout <<p<<" ";} std::cout << "\n";
        flag_work_ = false;
    }
}

void LwrTrajGenerator::getJointPosition(const sensor_msgs::JointState &msg){
    // the joint order is:
    // ['lwr_a1_joint', 'lwr_a2_joint', 'lwr_a3_joint', 'lwr_a4_joint', 'lwr_a5_joint', 'lwr_a6_joint', 'lwr_e1_joint']
    std::vector<size_t> joint_indexes = {0, 1, 3, 4, 5, 6, 2};
    for (size_t i = 0; i < joint_msr_pos_.size(); i++)
        joint_msr_pos_[joint_indexes[i]] =  msg.position[i];

}

