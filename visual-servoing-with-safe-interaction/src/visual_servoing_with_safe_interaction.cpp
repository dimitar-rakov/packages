
#include <visual_servoing_with_safe_interaction/visual_servoing_with_safe_interaction.h>
#include <utils/pseudo_inversion.h>
#include <utils/skew_symmetric.h>

#include <urdf/model.h>
#include <control_toolbox/filters.h>
#include <kdl_parser/kdl_parser.hpp>
#include <Eigen/LU>
#include <tf_conversions/tf_kdl.h>
#include <angles/angles.h>


VisualServoingWithSafeInteraction::VisualServoingWithSafeInteraction() {}
VisualServoingWithSafeInteraction::~VisualServoingWithSafeInteraction() {}


bool VisualServoingWithSafeInteraction::init(ros::NodeHandle &n)
{
  nh_ = n;
  urdf::Model model;


  safety_ton_obstacles_obj_.fromSec(-10.0);
  safety_ton_robot_obj_.fromSec(-10.0);
  safety_ton_msr_features_.fromSec(-10.0);
  safety_ton_des_features_.fromSec(-10.0);
  safety_ton_joints_state.fromSec(-10.0);;

  obstacles_obj_status_ = -1;
  robot_obj_status_ = -1;
  joints_state_status_ = -1;
  msr_features_status_ = -1;
  des_features_status_ = -1;

  cmd_flag_ = 0;
  num_released_features_ = 3;


  // Get base_name from parameter server
  if (!nh_.getParam("base_name", base_name_)){
    nh_.param("base_name", base_name_, std::string ("visual_servoing_with_safe_interaction"));
    ROS_WARN("Parameter base_name was not found. Default name is used: %s ", base_name_.c_str());
  }


  // Get base_name from parameter server
  if (!nh_.getParam("base_name", base_name_)){
    nh_.param("base_name", base_name_, std::string ("visual_servoing_with_safe_interaction"));
    ROS_WARN("Parameter base_name was not found. Default name is used: %s ", base_name_.c_str());
  }


  // get URDF and name of root and tip from the parameter server
  if (!nh_.getParam("root_name", root_name_)){
    ROS_ERROR_STREAM("VisualServoingWithSafeInteraction: No root name found on parameter server ("<<n.getNamespace()<<"/root_name)");
    return false;
  }

  if (!nh_.getParam("tip_name", tip_name_)){
    ROS_ERROR_STREAM("VisualServoingWithSafeInteraction: No tip name found on parameter server ("<<n.getNamespace()<<"/tip_name)");
    return false;
  }


  if (!ros::param::search(nh_.getNamespace()+"/robot_description", robot_description_)){
    ROS_ERROR_STREAM("VisualServoingWithSafeInteraction: No robot description (URDF) found on parameter server (/robot_description)");
    return false;
  }


  // Construct an URDF model from the xml string
  std::string xml_string;

  if (nh_.hasParam(robot_description_))
    nh_.getParam(robot_description_.c_str(), xml_string);
  else
  {
    ROS_ERROR("Parameter %s not set, shutting down node...", robot_description_.c_str());
    nh_.shutdown();
    return false;
  }

  if (xml_string.size() == 0)
  {
    ROS_ERROR("Unable to load robot model from parameter %s",robot_description_.c_str());
    nh_.shutdown();
    return false;
  }

  ROS_DEBUG("%s content\n%s", robot_description_.c_str(), xml_string.c_str());

  // Get urdf model out of robot_description

  if (!model.initString(xml_string))
  {
    ROS_ERROR("Failed to parse urdf file");
    n.shutdown();
    return false;
  }
  ROS_INFO("Successfully parsed urdf file in ");

  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromUrdfModel(model, kdl_tree))
  {
    ROS_ERROR("Failed to construct kdl tree");
    nh_.shutdown();
    return false;
  }

  // Populate the KDL chain
  if(!kdl_tree.getChain(root_name_, tip_name_, kdl_chain_))
  {
    ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
    ROS_ERROR_STREAM("  "<<root_name_<<" --> "<<tip_name_);
    ROS_ERROR_STREAM("  Tree has "<<kdl_tree.getNrOfJoints()<<" joints");
    ROS_ERROR_STREAM("  Tree has "<<kdl_tree.getNrOfSegments()<<" segments");
    ROS_ERROR_STREAM("  The segments are:");

    KDL::SegmentMap segment_map = kdl_tree.getSegments();
    KDL::SegmentMap::iterator it;

    for( it=segment_map.begin(); it != segment_map.end(); it++ )
      ROS_ERROR_STREAM( "    "<<(*it).first);

    return false;
  }

  ROS_DEBUG("Number of segments: %d", kdl_chain_.getNrOfSegments());
  ROS_DEBUG("Number of joints in chain: %d", kdl_chain_.getNrOfJoints());
  joint_number_  = kdl_chain_.getNrOfSegments();

  // Parsing joint limits from urdf model along kdl chain
  boost::shared_ptr<const urdf::Link> link = model.getLink(tip_name_);
  boost::shared_ptr<const urdf::Joint> joint;
  joint_limits_.min.resize(joint_number_);
  joint_limits_.max.resize(joint_number_);
  joint_limits_.center.resize(joint_number_);
  joint_limits_.velocity.resize(joint_number_);
  joint_limits_.acceleration.resize(joint_number_);
  joint_limits_.effort.resize(joint_number_);

  unsigned int index;
  for (unsigned int i = 0; i < joint_number_ && link; i++)
  {
    joint = model.getJoint(link->parent_joint->name);
    ROS_INFO("Getting limits for joint: %s", joint->name.c_str());
    traj_msg_.joint_names.insert(traj_msg_.joint_names.begin(),joint->name);
    tf_names_.insert(tf_names_.begin(),link->name);

    index = joint_number_ - i - 1;
    joint_limits_.min(index) = joint->limits->lower;
    joint_limits_.max(index) = joint->limits->upper;
    joint_limits_.center(index) = (joint_limits_.min(index) + joint_limits_.max(index))/2;
    joint_limits_.velocity(index) = joint->limits->velocity;
    joint_limits_.acceleration(index) = joint->limits->velocity *2.0; // maximum velocity can be achieved in 0.5 seconds
    joint_limits_.effort(index) = joint->limits->effort;
    link = model.getLink(link->getParent()->name);

  }
  tf_names_.insert(tf_names_.begin(),root_name_);


  // Get features_data_topic  from parameter server
  if (!nh_.getParam("features_data_topic", features_data_topic_)){
    nh_.param("features_data_topic", features_data_topic_, std::string ("/visual_features_extractor/visual_features_data"));
    ROS_WARN("Parameter features_data_topic was not found. Default topic's name is used: %s ", features_data_topic_.c_str());
  }

  // Get lwr_states_topic  from parameter server
  if (!nh_.getParam("joint_states_topic", joint_states_topic_)){
    nh_.param("joint_states_topic", features_data_topic_, std::string ("/joint_states"));
    ROS_WARN("Parameter joint_states_topic was not found. Default topic's name is used: %s ", joint_states_topic_.c_str());
  }

  // Get obstacles_objects_topic  from parameter server
  if (!nh_.getParam("obstacles_objects_topic", obstacles_objects_topic_)){
    nh_.param("obstacles_objects_topic", obstacles_objects_topic_, std::string ("/obstacle_detector/obstacle_objects"));
    ROS_WARN("Parameter obstacles_objects_topic was not found. Default topic's name is used: %s ", obstacles_objects_topic_.c_str());
  }

  // Get robot_objects_topic  from parameter server
  if (!nh_.getParam("robot_objects_topic", robot_objects_topic_)){
    nh_.param("robot_objects_topic", robot_objects_topic_, std::string ("/obstacle_detector/robot_objects"));
    ROS_WARN("Parameter robot_objects_topic was not found. Default topic's name is used: %s ", robot_objects_topic_.c_str());
  }

  // Get robot_command_topic  from parameter server
  if (!nh_.getParam("robot_command_topic", robot_command_topic_)){
    nh_.param("robot_command_topic", robot_command_topic_, std::string ("/lwr/joint_trajectory_controller/command"));
    ROS_WARN("Parameter robot_command_topic was not found. Default topic's name is used: %s ", robot_command_topic_.c_str());
  }

  // Get using_external_set_point  from parameter server
  if (!nh_.getParam("using_external_set_point", using_external_set_point_)){
    nh_.param("using_external_set_point", using_external_set_point_, false);
    ROS_WARN("Parameter using_external_set_point was not found. Default value is used: false");
  }

  // Get using_combined_matrices  from parameter server
  if (!nh_.getParam("using_combined_matrices", using_combined_matrices_)){
    nh_.param("using_combined_matrices", using_combined_matrices_, false);
    ROS_WARN("Parameter using_combined_matrices was not found. Default value is used: false");
  }

  // Get using_transpose_jacobian  from parameter server
  if (!nh_.getParam("using_transpose_jacobian", using_transpose_jacobian_)){
    nh_.param("using_transpose_jacobian", using_transpose_jacobian_, false);
    ROS_WARN("Parameter using_transpose_jacobian was not found. Default value is used: false");
  }

  // Get enb_obstacle_avoidance  from parameter server
  if (!nh_.getParam("enb_obstacle_avoidance", enb_obstacle_avoidance_)){
    nh_.param("enb_obstacle_avoidance", enb_obstacle_avoidance_, false);
    ROS_WARN("Parameter enb_obstacle_avoidance was not found. Default value is used: false");
  }

  // Get using_multiple_colision_points  from parameter server
  if (!nh_.getParam("using_multiple_colision_points", using_multiple_colision_points_)){
    nh_.param("using_multiple_colision_points", using_multiple_colision_points_, false);
    ROS_WARN("Parameter using_multiple_colision_points was not found. Default value is used: false");
  }


  gama_.resize(8);
  // Get gama from parameter server
  std::vector<double> gama;
  if (!nh_.getParam("gama", gama)){
    nh_.param("gama", gama, std::vector<double> {0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2});
    ROS_WARN("Parameter gama was not found. Default gain is used");
  }
  else if (gama.size() != 6 && gama.size() != 8){
    ROS_WARN("Parameter gama has wrong size: %ld. Default gain is used",gama.size());
    gama = {0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2};
  }


  for ( unsigned int  i = 0; i < gama.size(); i++)
    gama_(i) = gama[i];
  scaled_gama_ = gama_;


  joint_msr_states_.resize(joint_number_);
  joint_des_states_.resize(joint_number_);
  joint_des_states_prev_.resize(joint_number_);
  trajPoints_.positions.resize(joint_number_);
  trajPoints_.velocities.resize(joint_number_);

  jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
  fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
  Jbe_.resize(joint_number_);
  Jce_.resize(joint_number_);
  Jee_.resize(joint_number_);


  //Repulsive field init
  joint_rep_field_ = Eigen::VectorXd::Zero(7);
  joint_rep_field_prev_ = Eigen::VectorXd::Zero(7);


  sub_command_ = nh_.subscribe("command", 1, &VisualServoingWithSafeInteraction::command, this);
  sub_visual_features_data_ = nh_.subscribe(features_data_topic_, 1, &VisualServoingWithSafeInteraction::featureExtractorCB, this);
  sub_joint_state_= nh_.subscribe(joint_states_topic_, 1, &VisualServoingWithSafeInteraction::jointPositionCB, this);
  sub_markers_obstacle_objects_ = nh_.subscribe(obstacles_objects_topic_, 1, &VisualServoingWithSafeInteraction::obstacleObjectsCB, this);
  sub_markers_robot_objects_ = nh_.subscribe(robot_objects_topic_, 1, &VisualServoingWithSafeInteraction::robotObjectsCB, this);
  pub_all_data_ = nh_.advertise<std_msgs::Float64MultiArray>(nh_.getNamespace()+"/all_data", 5);
  pub_joint_traj_ = nh_.advertise<trajectory_msgs::JointTrajectory>( robot_command_topic_, 5 );
  pub_rep_vec_marker_ = nh_.advertise<visualization_msgs::Marker>(nh_.getNamespace()+"/repulsive_vector", 5 );
  ROS_INFO ("VisualServoingWithSafeInteraction with a name %s is initialized", base_name_.c_str());
  return true;
}

void VisualServoingWithSafeInteraction::starting(const ros::Time& time) {


  cmd_flag_ = 1;
  ROS_INFO_STREAM("joint_msr_states.q: "<<joint_msr_states_.q.data.transpose());
  ROS_INFO_STREAM("joint_des_states.q: "<<joint_des_states_.q.data.transpose());
  trajPoints_.time_from_start.fromSec(0.02);
  trajPoints_.velocities.clear();
  trajPoints_.accelerations.clear();
  trajPoints_.effort.clear();
  fk_pos_solver_->JntToCart(joint_msr_states_.q, Fbe_des_);
  for ( unsigned int  i = 0; i < joint_number_; i++)
    trajPoints_.positions[i] = joint_des_states_.q(i);
}

void VisualServoingWithSafeInteraction::update(const ros::Time& time, const ros::Duration& period)
{

  // Safety timers
  if ((ros::Time::now()- safety_ton_obstacles_obj_).toSec()< 2.0 && obstacles_obj_status_> -1){ obstacles_obj_status_= 1; }
  else if ((ros::Time::now()- safety_ton_obstacles_obj_).toSec()> 2.0 && obstacles_obj_status_> -1){ obstacles_obj_status_= 0; }
  if (obstacles_obj_status_ == 0) ROS_WARN("Obstacle objects's topic is not longer available");
  else if (obstacles_obj_status_ == -1  && enb_obstacle_avoidance_) ROS_WARN_THROTTLE(5, "Waiting for obstacle objects's topic");

  if ((ros::Time::now()- safety_ton_robot_obj_).toSec()< 2.0 && robot_obj_status_> -1){ robot_obj_status_= 1; }
  else if ((ros::Time::now()- safety_ton_robot_obj_).toSec()> 2.0 && robot_obj_status_> -1){ robot_obj_status_= 0; }
  if (robot_obj_status_ == 0) ROS_WARN("Robot objects's topic is not longer available");
  else if (robot_obj_status_ == -1 && enb_obstacle_avoidance_) ROS_WARN_THROTTLE(5, "Waiting for robot objects's topic");


  if ((ros::Time::now()- safety_ton_msr_features_).toSec()> 2.0 && msr_features_status_> -1){ msr_features_status_= 0; }
  if (msr_features_status_ == 0) ROS_WARN("Measured features's topic is not longer available");
  else if (msr_features_status_ == -1) ROS_WARN_THROTTLE(5, "Waiting for measured features's topic");
  else if (msr_features_status_ == 1) ROS_WARN_THROTTLE(5, "Measured features are out of FOV");

  if ((ros::Time::now()- safety_ton_des_features_).toSec()> 2.0 && des_features_status_> -1){ des_features_status_= 0; }
  if (des_features_status_ == 0) ROS_WARN("Desired features's topic is not longer available");
  else if (des_features_status_ == -1) ROS_WARN_THROTTLE(5, "Waiting for desired features's topic");
  else if (des_features_status_ == 1) ROS_WARN_THROTTLE(5, "Desired features are out of FOV");

  // use   msr joint positions  to
  if (joints_state_status_ == 2 && getTFs())
  {
    // computing forward kinematics
    fk_pos_solver_->JntToCart(joint_msr_states_.q, Fbe_);

    // computing Jacobian
    jnt_to_jac_solver_->JntToJac(joint_msr_states_.q, Jbe_);

    // Jacobian camera with respect to the camera Jcc
    /// ref "Comparison of Basic Visual Servoing Methods" F. Janabi-Sharifi and L. Deng and W. J. Wilson
    ///
    ///     KDL::Jacobian Jee (joint_number_), Jce (joint_number_);
    ///     Eigen::MatrixXd Xeb = Eigen::MatrixXd::Identity(6,6);
    ///     Eigen::MatrixXd Xce = Eigen::MatrixXd::Identity(6,6);
    ///     Map<Matrix<double, 3, 3, RowMajor> > Reb(Fbe_.Inverse().M.data,3,3);
    ///     Xeb.topLeftCorner(3,3) = Reb;
    ///     Xeb.bottomRightCorner(3,3) = Reb;
    ///     Eigen::Matrix<double,3,3> p_hat;
    ///     KDL::Frame Fce =Fec_.Inverse();
    ///     Map<Matrix<double, 3, 3, RowMajor> > Rce(Fce.M.data,3,3);
    ///     skew_symmetric (Fce.p , p_hat);
    ///     Xce.topLeftCorner(3,3) = Rce;
    ///     Xce.topRightCorner(3,3) = p_hat*Rce;
    ///     Xce.bottomRightCorner(3,3) = Rce;
    ///     Jee.data = Xeb*Jbe_.data;
    ///     Jce.data = Xce*Jee.data;

    KDL::changeBase(Jbe_, Fbe_.Inverse().M, Jee_);
    KDL::changeRefFrame(Jee_, Fec_.Inverse(), Jce_);

    // computing J_pinv_ce_
    pseudo_inverse(Jce_.data, J_pinv_ce_);

    if (msr_features_status_ ==2 && des_features_status_ ==2 && cmd_flag_ ==1 && !TEST_AVOIDANCE)
    {
//      // Simple Gain Scheduling up to 3 times
//      if ( (s_des_- s_msr_).squaredNorm() >  0.000002 && (s_des_- s_msr_).squaredNorm() <  0.002)
//        scaled_gama_ =  gama_ * (1 + 1 * 0.000002 / (s_des_- s_msr_).squaredNorm());
//      else if ((s_des_- s_msr_).squaredNorm() <=  0.000002)
//        scaled_gama_ =  gama_ ;
//      else if ((s_des_- s_msr_).squaredNorm() >=  0.002)
//        scaled_gama_ =  gama_;

      ROS_INFO_STREAM("Norm :"<<(s_des_- s_msr_).squaredNorm()<< "scaled_gama_(0,0): "<<scaled_gama_(0,0));

      // if it used external setpoint Lhat_msr_ = Lhat_des_
      if (using_combined_matrices_)
        Lhat_ = 0.5 * (Lhat_msr_ + Lhat_des_);
      else
        Lhat_ = Lhat_msr_;
      s_err_dot_.resize(s_msr_.size());
      for (int i = 0 ; i < s_msr_.size(); i++){
        s_err_dot_(i) = -scaled_gama_(i) *(s_msr_(i) - s_des_(i));
        // Moment based VS => s(5) is alpha
        if (i == 5 && s_msr_.size() == 6 )
          s_err_dot_(i) = -scaled_gama_(i) * angles::shortest_angular_distance(s_msr_(i), s_des_(i));
      }

//      s_err_dot_(0) = 0.0;
//      s_err_dot_(1) = 0.0;
//      s_err_dot_(2) = 0.0;
//      s_err_dot_(3) = 0.0;
//      s_err_dot_(4) = 0.0;
//      s_err_dot_(5) = 0.0;




      pseudo_inverse(Lhat_*Jce_.data, Lhat_Jce_pinv_);
      joint_des_states_prev_.q.data = joint_des_states_.q.data;
      joint_des_states_prev_.qdot.data = joint_des_states_.qdot.data;

      if (enb_obstacle_avoidance_ && obstacles_obj_status_ == 1 && robot_obj_status_ == 1){
        s_err1_dot_ = s_err_dot_.head(s_err_dot_.size() - num_released_features_);
        s_err2_dot_ = s_err_dot_.tail(num_released_features_);
        L1hat_ = Lhat_.topRows(s_msr_.size() - num_released_features_);
        L2hat_ = Lhat_.bottomRows(num_released_features_);
        pseudo_inverse(L1hat_*Jce_.data, L1hat_Jce_pinv_);

        // Safety part
        double min_dist ,max_ni;
        obstaclesProcesing(min_dist , max_ni);
        ROS_WARN_COND(min_dist < ro1_,"OBSTACLES AVOIDANCE ACTIVE");
        s_err_obs2_dot_ = ((1 - max_ni) * s_err2_dot_) + max_ni * (L2hat_* Jce_.data * L1hat_Jce_pinv_) * s_err1_dot_ ;
        Eigen::MatrixXd N = (Eigen::MatrixXd::Identity(joint_number_,joint_number_) - L1hat_Jce_pinv_* (L1hat_*Jce_.data));
        s_err_obs_dot_ << s_err1_dot_, s_err_obs2_dot_;
        joint_des_states_.qdot.data =  (Lhat_Jce_pinv_ * s_err_obs_dot_) + (max_ni * N * joint_rep_field_);
        //joint_des_states_.qdot.data =  (L1hat_Jce_pinv_ * s_err1_dot_) + (N * joint_rep_field_);
      }
      else
        joint_des_states_.qdot.data =  Lhat_Jce_pinv_ * (s_err_dot_);


      // Diffferent on target criteria depending of type of s
      if (s_msr_.size() ==6 && ( s_des_- s_msr_).squaredNorm() <  0.000001){
        ROS_INFO("On target");
        //cmd_flag_ = 0;
      }

      if (s_msr_.size() ==8 && ( s_des_- s_msr_).squaredNorm() <  0.00001){
        ROS_INFO("On target");
        cmd_flag_ = 0;
      }

      // Record data at interval 10 ms
      if (record_interval_>= 0.010){
        recordAllData();
        record_interval_ = 0.0;
      }
      else
        record_interval_ += period.toSec();
    }
    else if (TEST_AVOIDANCE){
      cmd_flag_ = 1;
      testAvoidance(period);
    }

    // integrating q_dot -> getting q (Trapezoidal method)
    for ( unsigned int  i = 0; i < joint_number_; i++){
      joint_des_states_.q(i) += period.toSec()*0.5*(joint_des_states_prev_.qdot(i)+joint_des_states_.qdot(i));

      // Limit maximmum desired joint step send to robot and max joint position
      double err_dist = 0.0;
      angles::shortest_angular_distance_with_limits(joint_des_states_prev_.q(i), joint_des_states_.q(i), joint_limits_.min(i), joint_limits_.max(i), err_dist);
      if(err_dist >  MAX_JOINT_DES_STEP)
         joint_des_states_.q(i) = joint_des_states_prev_.q(i) + MAX_JOINT_DES_STEP;
      if(err_dist < -MAX_JOINT_DES_STEP)
         joint_des_states_.q(i) = joint_des_states_prev_.q(i) - MAX_JOINT_DES_STEP;

      if (joint_des_states_.q(i) < joint_limits_.min(i) + 0.01){
        joint_des_states_.q(i) = joint_limits_.min(i) + 0.01;
        ROS_WARN_DELAYED_THROTTLE(5,"Activated min position saturation for joint %u", i);
      }
      if (joint_des_states_.q(i) > joint_limits_.max(i) - 0.01){
        joint_des_states_.q(i) = joint_limits_.max(i) - 0.01;
        ROS_WARN_DELAYED_THROTTLE(5,"Activated max position saturation for joint %u", i);
      }
    }

    // set controls for joints
    for (unsigned int  i = 0; i < joint_number_; i++)
        trajPoints_.positions[i] = joint_des_states_.q(i);
    ROS_INFO("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");

    trajPoints_.time_from_start = period;
    traj_msg_.points.clear();
    traj_msg_.points.push_back(trajPoints_);

  }
}

void VisualServoingWithSafeInteraction::command(const visual_servoing_with_safe_interaction::VisualServoing &msg)
{
  //cmd_mutex_.lock();
  for (size_t i = 0; i < msg.s_des.size(); i++)
  {
    if (using_external_set_point_){
      // ToDo consider removing
      //            s_des_[i]= msg.s_des[i];
      //            des_features_status_ = 2 ;
      //            safety_ton_des_features_ = ros::Time::now();
    }
  }
  //cmd_mutex_.unlock();
  ROS_INFO_STREAM("New setpoint for s is: "<<s_des_.transpose());
  cmd_flag_ = 1;
}

void VisualServoingWithSafeInteraction::publish(){
  if (!traj_msg_.points.empty())
    pub_joint_traj_.publish(traj_msg_);
  if (!all_data_msg.data.empty())
    pub_all_data_.publish (all_data_msg);
}

void VisualServoingWithSafeInteraction::featureExtractorCB(const visual_servoing_with_safe_interaction::VisFeature &msg){

  if (msg.is_valid_msr_feature ){

    // First time received data initializes the size of the used matrices and the gain
    if (msr_features_status_ ==-1 || s_msr_.size() < 6 )
    {
      s_msr_ = VectorXd::Zero(msg.size_s_msr);
      s_err_obs_dot_  = VectorXd::Zero(msg.size_s_msr);
      Lhat_msr_ = MatrixXd::Zero(msg.rowsLmsr, msg.colsLmsr);
      ROS_INFO("First msr features data is received");
    }

    // Exponential smoothing filter. Values closer to 0 weight the last smoothed value more heavily
    for (size_t i = 0; i< msg.size_s_msr; i++){
      if (msr_features_status_ ==-1)
        s_msr_(i) = msg.s_msr[i];
      else
        s_msr_(i) = filters::exponentialSmoothing(msg.s_msr[i], s_msr_(i), 0.5);
    }
    for (size_t r = 0; r< msg.rowsLmsr; r++){
      for (size_t c = 0; c< msg.colsLmsr; c++)
        Lhat_msr_(r, c) = msg.L_data_msr[r*msg.colsLmsr + c];
    }

    // for initialization at start set des = msr otherwise is overlapped
    if (des_features_status_ == -1  || s_des_.size() < 6 ) {
      s_des_ = s_msr_;
      Lhat_des_ = Lhat_msr_;
    }

    if (msg.is_valid_des_feature){
      for (size_t i = 0; i< msg.size_s_des; i++)
        s_des_(i) = msg.s_des[i];

      for (size_t r = 0; r< msg.rowsLmsr; r++){
        for (size_t c = 0; c< msg.colsLmsr; c++){
          if (msg.is_valid_des_feature)
            Lhat_des_(r, c) = msg.L_data_des[r*msg.colsLdes + c];
        }
      }
    }
    //        std::cout << std::fixed << std::setprecision(4);
    //        std::cout << "\nData in s_msr:"<<"\n" <<s_msr_.transpose();
    //        std::cout << "\nData in s_des"<<"\n" <<s_des_.transpose();
    //        std::cout << "\nData in L_msr:"<<"\n" <<Lhat_msr_;
    //        std::cout << "\nData in L_des:"<<"\n" <<Lhat_des_<< "\n";
  }
  msr_features_status_ = (msg.is_valid_msr_feature)? 2 : 1;
  safety_ton_msr_features_ = ros::Time::now();
  des_features_status_ = (msg.is_valid_des_feature)? 2 : 1;
  safety_ton_des_features_ = ros::Time::now();

}

void VisualServoingWithSafeInteraction::obstacleObjectsCB(const visualization_msgs::MarkerArray &msg){
  //obs_mutex_.lock();
  markers_obstacle_objects_ = msg;
  obstacles_obj_status_ = 1;
  safety_ton_obstacles_obj_ = ros::Time::now();
  //obs_mutex_.unlock();
}

void VisualServoingWithSafeInteraction::robotObjectsCB(const visualization_msgs::MarkerArray &msg){
  //robot_mutex_.lock();
  markers_robot_objects_ = msg;
  robot_obj_status_ = 1;
  safety_ton_robot_obj_ = ros::Time::now();
  //robot_mutex_.unlock();
}

void VisualServoingWithSafeInteraction::recordAllData(){

  // All data for rosbag
  all_data_msg.data.clear();
  //measured angles 3-9
  for (unsigned int i = 0; i < joint_number_; i++)
    all_data_msg.data.push_back(joint_msr_states_.q(i));

  //desired angles 10-16
  for (unsigned int i = 0; i < joint_number_; i++)
    all_data_msg.data.push_back(joint_des_states_.q(i));

  //measured velocity 17-23
  for (unsigned int i = 0; i < joint_number_; i++)
    all_data_msg.data.push_back(joint_msr_states_.qdot(i));

  //desired velocity 24-30
  for (unsigned int i = 0; i < joint_number_; i++)
    all_data_msg.data.push_back(joint_des_states_.qdot(i));

  //repulsive field velocity(simple 31-37 )
  for (unsigned int i = 0; i < joint_rep_field_.size(); i++)
    all_data_msg.data.push_back(joint_rep_field_(i));


  // Fbe_ 38-43
  double roll, pitch, yaw;
  KDL::Frame Fwc = Fwb_ * Fbe_ * Fec_;
  all_data_msg.data.push_back(Fwc.p.x());
  all_data_msg.data.push_back(Fwc.p.y());
  all_data_msg.data.push_back(Fwc.p.z());
  Fwc.M.RPY(roll, pitch, yaw);
  all_data_msg.data.push_back(roll);
  all_data_msg.data.push_back(pitch);
  all_data_msg.data.push_back(yaw);

  //measured features vector (simple 44-49 )
  for (unsigned int i = 0; i < s_msr_.size(); i++)
    all_data_msg.data.push_back(s_msr_(i));

  //desired features vector (simple 50-55 )
  for (unsigned int i = 0; i < s_des_.size(); i++)
    all_data_msg.data.push_back(s_des_(i));


  //error features vector_dot (simple 56-61 )
  for (unsigned int i = 0; i < s_err_dot_.size(); i++)
    all_data_msg.data.push_back(s_err_dot_(i));

  // Calculate svd of the Jbc_ (simple 62-68 )
  JacobiSVD<MatrixXd> svd1(Jce_.data, ComputeThinU | ComputeThinV);



  Eigen::IOFormat CleanFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n", "[", "]");
  std::cout << std::fixed << std::setprecision(4);
  ROS_WARN_STREAM_COND(svd1.singularValues()(5) < 0.001,"Jce_ sigmas "<<std::fixed << std::setprecision(4)<<(svd1.singularValues()).transpose().format(CleanFmt));
  for (long i = 0; i < (svd1.singularValues()).size(); i++)
    all_data_msg.data.push_back((svd1.singularValues())(i));

  // Calculate svd of the  Lhat_*Jce_ (simple 69-75 )
  JacobiSVD<MatrixXd> svd2((Lhat_*Jce_.data), ComputeFullU | ComputeFullV);
  ROS_WARN_STREAM_COND(svd2.singularValues()(5) < 0.001,"Lhat_*Jce_ sigmas: "<<std::fixed << std::setprecision(4)<<(svd2.singularValues()).transpose().format(CleanFmt));
  for (long i = 0; i < (svd2.singularValues()).size(); i++)
    all_data_msg.data.push_back((svd2.singularValues())(i));


  // ToDo remove
  for (size_t i = 0; i < joint_number_; i++)
    all_data_msg.data.push_back(trajPoints_.positions[i]);
}

void VisualServoingWithSafeInteraction::jointPositionCB(const sensor_msgs::JointState &msg){
  // joint order is ['lwr_a1_joint', 'lwr_a2_joint', 'lwr_a3_joint', 'lwr_a4_joint', 'lwr_a5_joint', 'lwr_a6_joint', 'lwr_e1_joint']
  joint_mutex_.lock();
  int cnt_founded_joints = 0;
  for (unsigned int i = 0; i < traj_msg_.joint_names.size(); i++){
    for (unsigned int j = 0; j < msg.name.size(); j++){
      if ( traj_msg_.joint_names[i] == msg.name[j]){
        joint_msr_states_.q(i) =  msg.position[j];
        joint_msr_states_.qdot(i) =  msg.velocity[j];

        // fill desired joint position and velocities at the starting
        if (joints_state_status_ == -1){
          joint_des_states_.q(i) = joint_msr_states_.q(i);
          joint_des_states_.qdot(i) = 0.0;
          joint_des_states_prev_.q(i) = joint_des_states_.q(i);
          joint_des_states_prev_.qdot(i) = 0.0;
        }
        cnt_founded_joints++;
      }
    }
  }
  if (joints_state_status_ == -1)
    starting(ros::Time::now());
  joints_state_status_ = 1;
  joint_mutex_.unlock();
  if (cnt_founded_joints != static_cast<int>(joint_number_))
    ROS_WARN ("Joint names in joint_states topic do not corespond to joint names in robot model");
  else
    joints_state_status_ = 2;
}

void VisualServoingWithSafeInteraction::obstaclesProcesing(double &dst_min_dist, double &dst_max_ni){

  KDL::Frame Fwo, Fbj, Fjs;
  KDL::Jacobian Jbj(joint_number_), Jbs(joint_number_);
  std::vector< size_t > joint_to_base_indeces;
  Fbs_.clear();
  Fbo_.clear();


  for (size_t i = 0; i < markers_obstacle_objects_.markers.size(); i++){
    tf::poseMsgToKDL(markers_obstacle_objects_.markers[i].pose, Fwo);
    Fbo_.push_back(Fwb_.Inverse() * Fwo);
  }


  for(size_t i = 0; i< markers_robot_objects_.markers.size(); i++){
    for(size_t j = 0; j< tf_names_.size(); j++){
      if (tf_names_[j] == markers_robot_objects_.markers[i].header.frame_id){
        fk_pos_solver_->JntToCart(joint_msr_states_.q, Fbj, static_cast<int>(j));
        joint_to_base_indeces.push_back(j);
        tf::poseMsgToKDL(markers_robot_objects_.markers[i].pose, Fjs);
        Fbs_.push_back(Fbj * Fjs);
      }
    }
  }


  KDL::Vector min_dist_vector;
  KDL::Vector Pbo_surface_max, Pbs_surface_max, Pjs_surface;
  Eigen::VectorXd Vrep = Eigen::VectorXd::Zero(6);
  joint_rep_field_prev_ = joint_rep_field_;
  joint_rep_field_ = Eigen::VectorXd::Zero(7);
  double min_dist = 10000; //initialize with 10000 meters away
  double ni = 0.0;
  double max_ni = 0.0;
  size_t idx_closest_bj = 0;

  for (size_t i = 0; i< markers_obstacle_objects_.markers.size(); i++){

    // radius obstacle object, always approximate to sphere with largest axis
    double ro_sx = markers_obstacle_objects_.markers[i].scale.x;
    double ro_sy = markers_obstacle_objects_.markers[i].scale.y;
    double ro_sz = markers_obstacle_objects_.markers[i].scale.z;
    double ro2 = 0.5 * std::max(std::max(ro_sx, ro_sy),  ro_sz);

    for (size_t j = 0; j< Fbs_.size(); j++){
      // radius robot sphere always approximate to sphere with largest axis
      double os_sx = markers_robot_objects_.markers[j].scale.x;
      double os_sy = markers_robot_objects_.markers[j].scale.y;
      double os_sz = markers_robot_objects_.markers[j].scale.z;
      double rs2 = 0.5 *std::max(std::max(os_sx, os_sy),  os_sz);

      // in obs_position base coordinates ToDo check frames id
      double dist_centers = std::abs((Fbo_[i].p - Fbs_[j].p).Norm());

      // point on body sphere where the line between two objects centers cross the body sphere surface
      KDL::Vector Pbs_surface =  Fbs_[j].p + rs2/dist_centers * (Fbo_[i].p - Fbs_[j].p );

      // point on obstacle sphere where the line between two objects centers cross the obstacle sphere surface
      KDL::Vector Pbo_surface =  Fbo_[i].p + ro2/dist_centers * (Fbs_[j].p - Fbo_[i].p);
      KDL::Vector dist_vec_surfaces = Pbo_surface - Pbs_surface;
      double dist_surfaces = std::abs((dist_vec_surfaces).Norm());

      ni = Vmax_/(1 + std::exp((std::abs(dist_surfaces) *2/ro1_ - 1) * alfa_));
      if (min_dist > dist_surfaces){
        min_dist = dist_surfaces;
        max_ni = ni;
        idx_closest_bj = joint_to_base_indeces[j];
        min_dist_vector = dist_vec_surfaces;
        Pbo_surface_max = Pbo_surface;
        Pbs_surface_max = Pbs_surface;
      }

      if (using_multiple_colision_points_){
        fk_pos_solver_->JntToCart(joint_msr_states_.q, Fbj, static_cast<int>(joint_to_base_indeces[j]));
        jnt_to_jac_solver_->JntToJac(joint_msr_states_.q, Jbj, static_cast<int>(joint_to_base_indeces[j]));
        Pjs_surface = Fbj.Inverse() * Pbs_surface;
        KDL::changeRefPoint(Jbj, Pjs_surface, Jbs);

        // use distance vector as linear velocity and set the angular velocity to zero
        Vrep = (ni / dist_centers) * (Eigen::MatrixXd(6,1) << dist_vec_surfaces(0), dist_vec_surfaces(1), dist_vec_surfaces(2), 0.0, 0.0, 0.0).finished();
        if (using_transpose_jacobian_)
          joint_rep_field_ += - Jbs.data.transpose() * Vrep;
        else {
          pseudo_inverse(Jbs.data, J_pinv_bs_);
          joint_rep_field_ += - J_pinv_bs_ * Vrep;      // in original paper is with -J_pinv_bs_ * Vrep
        }
      }
    }
  }

  if (!using_multiple_colision_points_){
    // reduce calculations if only one point is used
    fk_pos_solver_->JntToCart(joint_msr_states_.q, Fbj, static_cast<int>(idx_closest_bj));
    jnt_to_jac_solver_->JntToJac(joint_msr_states_.q, Jbj, static_cast<int>(idx_closest_bj));
    Pjs_surface = Fbj.Inverse() * Pbs_surface_max;
    KDL::changeRefPoint(Jbj, Pjs_surface, Jbs);

    // use distance vector as linear velocity and set the angular velocity to zero
    Vrep = (max_ni / min_dist) * (Eigen::MatrixXd(6,1) << min_dist_vector(0), min_dist_vector(1), min_dist_vector(2), 0.0, 0.0, 0.0).finished();
    if (using_transpose_jacobian_)
      joint_rep_field_ = -Jbs.data.transpose() * Vrep;
    else {
      pseudo_inverse(Jbs.data, J_pinv_bs_);
      joint_rep_field_ = -J_pinv_bs_ * Vrep;      // in original paper is with -J_pinv_bs_ * Vrep
    }
  }

  // Exponential smoothing filter for joint_rep_field_. Values closer to 0 weight the last smoothed value more heavily
  for (long i = 0; i< joint_rep_field_.size(); i++)
    joint_rep_field_(i) = filters::exponentialSmoothing(joint_rep_field_(i), joint_rep_field_prev_(i), 0.1);
  dst_min_dist = min_dist;
  dst_max_ni = max_ni;

  // Publish the vector which repesents the closest to robot obstacle
  visualization_msgs::Marker marker;
  geometry_msgs::Point p;
  marker.header.frame_id = "lwr_base_link";
  marker.header.stamp = ros::Time();
  marker.ns = "repulsive vector";
  marker.id = 348;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x =  0.02;
  marker.scale.y =  0.03;
  marker.scale.z =  0.05;
  marker.color.a = 0.75; // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.lifetime = ros::Duration(1);
  tf::pointKDLToMsg(Pbo_surface_max, p);
  marker.points.clear();
  marker.points.push_back(p);
  tf::pointKDLToMsg(Pbs_surface_max, p);
  marker.points.push_back(p);
  pub_rep_vec_marker_.publish(marker);
}

bool VisualServoingWithSafeInteraction::getTFs(){
  tf::StampedTransform transform;
  try{
    lr_.lookupTransform("lwr_a6_link", "eye_in_hand", ros::Time(0), transform);
    tf::transformTFToKDL(transform, Fec_);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
    return false;
  }

  try{
    lr_.lookupTransform("world", "lwr_base_link", ros::Time(0), transform);
    tf::transformTFToKDL(transform, Fwb_);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
    return false;
  }
  return  true;

}

 void VisualServoingWithSafeInteraction::testAvoidance(ros::Duration period){

   if (cmd_flag_ ==1)
   {

     Eigen:: MatrixXd Jbe_pin, Jbe1, Jbe2,  Jbe1_pin, Jbe2_pin;
     Eigen::VectorXd x_err_dot, x_err1_dot, x_err2_dot, x_err_obs_dot, x_err_obs2_dot;
     pseudo_inverse(Jbe_.data, Jbe_pin);
     joint_des_states_prev_.q.data = joint_des_states_.q.data;
     joint_des_states_prev_.qdot.data = joint_des_states_.qdot.data;
     KDL::Twist x_err_ = diff(Fbe_, Fbe_des_);
     x_err_dot.resize(6);
     x_err_obs_dot.resize(6);
     x_err_dot <<  x_err_.vel.x(), x_err_.vel.y(), x_err_.vel.z(), x_err_.rot.x(), x_err_.rot.y(), x_err_.rot.z();

     if (enb_obstacle_avoidance_ && obstacles_obj_status_ == 1 && robot_obj_status_ == 1){
       x_err1_dot = x_err_dot.head(x_err_dot.size() - num_released_features_);
       x_err2_dot = x_err_dot.tail(num_released_features_);
       Jbe1 = Jbe_.data.topRows(x_err1_dot.size());
       Jbe2 = Jbe_.data.bottomRows(x_err2_dot.size());
       pseudo_inverse(Jbe1, Jbe1_pin);

       // Safety part
       double min_dist ,max_ni;
       obstaclesProcesing(min_dist , max_ni);

       ROS_WARN_COND(min_dist < ro1_,"OBSTACLES AVOIDANCE ACTIVE");

       x_err_obs2_dot = ((1 - max_ni) * x_err2_dot) + max_ni * (Jbe2 * Jbe1_pin) * x_err1_dot ;
       Eigen::MatrixXd N = (Eigen::MatrixXd::Identity(joint_number_,joint_number_) - Jbe1_pin* (Jbe1));
       x_err_obs_dot << x_err1_dot, x_err_obs2_dot;
       if(min_dist < ro1_){
        //joint_des_states_.qdot.data =  (Jbe_pin * x_err_obs_dot) + (max_ni * N * joint_rep_field_);
        joint_des_states_.qdot.data =  joint_rep_field_;
       }
       else
        joint_des_states_.qdot.data =  ( Jbe_pin * x_err_dot);
     }

     if (Equal(Fbe_, Fbe_des_, 0.005))
     {
       ROS_INFO("On target");
     }

   }
 }
