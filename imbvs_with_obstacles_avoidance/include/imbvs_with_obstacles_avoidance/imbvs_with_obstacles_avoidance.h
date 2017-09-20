#ifndef IIMBVS_WITH_OBSTACLES_AVOIDANCE
#define IIMBVS_WITH_OBSTACLES_AVOIDANCE


#include "imbvs_with_obstacles_avoidance/VisualServoing.h"
#include "imbvs_with_obstacles_avoidance/PoseRPY.h"
#include <imbvs_with_obstacles_avoidance/SetGama.h>
#include "imbvs_with_obstacles_avoidance/VisFeature.h"


#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/chaindynparam.hpp> //this to compute the gravity vector
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>

// TF
#include <tf/transform_listener.h>
#include "tf/transform_datatypes.h"
#include <tf_conversions/tf_eigen.h>
#include <tf_conversions/tf_kdl.h>
#include <eigen_conversions/eigen_kdl.h>

#include <Eigen/Dense>



/// SAFETY maximum difference between two desire joint position
const static double MAX_JOINT_DES_STEP = 0.0025;
/// using of trajectory client instead of oj just publisher
const static bool USE_TRAJ_CLIENT = true;

class ImbvsWithObstaclesAvoidance
{
public:

    boost::mutex   features_mutex_;
    boost::mutex   obs_mutex_;
    boost::mutex   cmd_mutex_;
    boost::mutex   joint_mutex_;
    boost::mutex   robot_mutex_;
    ImbvsWithObstaclesAvoidance();
    ~ImbvsWithObstaclesAvoidance();

    bool init(ros::NodeHandle &n);
    // one time called before first update
    void starting(const ros::Time& time);
    // all calculations
    void update(const ros::Time& time, const ros::Duration& period);
    //external setpoint for desired features (not used)
    void command(const  imbvs_with_obstacles_avoidance::VisualServoing &msg);
    // all publishers
    void publish();


private:

    ros::NodeHandle nh_;
    KDL::Chain kdl_chain_;
    KDL::JntArrayAcc joint_msr_states_, joint_des_states_, joint_des_states_prev_;  // joint states (measured , desired )
    KDL::JntArrayAcc joint_init_interaction_states_;        // stored joint states at initial obstacles interaction
    struct limits_
    {
        KDL::JntArray min;
        KDL::JntArray max;
        KDL::JntArray center;
        KDL::JntArray velocity;
        KDL::JntArray acceleration;
        KDL::JntArray jerk;
        KDL::JntArray effort;
    } joint_limits_;

    ros::Subscriber sub_command_;
    ros::Subscriber sub_gains_;
    ros::Subscriber sub_visual_features_data_;
    ros::Subscriber sub_markers_obstacle_objects_;
    ros::Subscriber sub_markers_robot_objects_;
    ros::Subscriber sub_joint_state_;  
    ros::Publisher  pub_all_data_;
    ros::Publisher  pub_joint_traj_;
    boost::shared_ptr <actionlib::SimpleActionClient< control_msgs::FollowJointTrajectoryAction > > traj_client_ptr_;
    ros::Publisher  pub_rep_vec_marker_;
    tf::TransformListener lr_;


    std_msgs::Float64MultiArray all_data_msg;
    trajectory_msgs::JointTrajectoryPoint trajPoints_;
    trajectory_msgs::JointTrajectory traj_msg_;
    std::vector<std::string> tf_names_;
    visualization_msgs::MarkerArray markers_robot_objects_;
    visualization_msgs::MarkerArray markers_obstacle_objects_;

    KDL::Frame Fbe_;                            //current pose end efffector wrt robot base
    KDL::Frame Fwe_;                            //end efffector wrt robot world
    KDL::Frame Fec_;                            //camera wrt to end effector
    KDL::Frame Fwb_;                            //base wrt to world
    std::vector< KDL::Frame> Fbo_;              //poses of obstacles wrt base
    std::vector< KDL::Frame> Fbs_;              //poses of robot objects wrt base

    //points on obstacles object surface, where distance to robot object surface is min
    std::vector<KDL::Vector> Pb_os_min_;
    //points on robot object surface, where distance to obstacles surface is min
    std::vector<KDL::Vector> Pb_rs_min_;

    KDL::Jacobian Jbe_, Jce_, Jee_;             //Jacobians
    boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;

    Eigen::MatrixXd J_pinv_ce_, J_pinv_bs_ ;
    Eigen::VectorXd joint_rep_field_, joint_rep_field_prev_;


    Eigen::VectorXd s_msr_, s_des_, s_err_obs_dot_;
    Eigen::VectorXd s_err_dot_, s_err1_dot_, s_err2_dot_, s_err_obs2_dot_ ;
    Eigen::MatrixXd Lhat_msr_, Lhat_des_;
    Eigen::MatrixXd Lhat_, L1hat_, L2hat_;
    Eigen::VectorXd gama_, scaled_gama_;
    Eigen::MatrixXd Lhat_Jce_pinv_;
    Eigen::MatrixXd L1hat_Jce_pinv_;
    int num_released_features_;
    double min_dist_, max_ni1_;


    // flags for topics data. status -1 - not not received, status 0 - delayed,
    // status 1 - receive in time and data ok, status 2 - receive in time and not valid data,
    int obstacles_obj_status_;
    int robot_obj_status_;
    int msr_features_status_;
    int des_features_status_;
    int joints_state_status_;


    // Timers
    ros::Time safety_ton_robot_obj_, safety_ton_obstacles_obj_, safety_ton_msr_features_;
    ros::Time safety_ton_des_features_, safety_ton_joints_state;


    unsigned int joint_number_;
    int cmd_flag_;
    double record_interval_;
    bool interaction_active_ ;
    bool restoring_initial_ ;


    // Parameters server variable
    bool using_combined_matrices_;
    bool using_external_set_point_;
    bool using_transpose_jacobian_;
    bool using_multiple_colision_points_;
    bool enb_obstacle_avoidance_;
    std::string base_name_, robot_description_, root_name_, tip_name_;
    std::string features_data_topic_, obstacles_objects_topic_, robot_objects_topic_;
    std::string joint_states_topic_, robot_command_topic_, robot_trajectory_action_topic_;
    bool test_only_obst_avoidance_;
    double ro1_, ro2_;
    double Vmax_;
    double alpha_;






    /// Temp
    KDL::Frame Fbe_des_;


    ///

    void jointPositionCB(const sensor_msgs::JointState &msg);

    void featureExtractorCB(const imbvs_with_obstacles_avoidance::VisFeature &msg);

    void obstacleObjectsCB(const visualization_msgs::MarkerArray &msg);

    void robotObjectsCB(const visualization_msgs::MarkerArray &msg);

    void recordAllData();

    void obstaclesProcesing(double &dst_min_dist, double &dst_max_ni1);


    void calcObjectsFrames(std::vector<size_t> &joint_to_base_indeces);

    void calcMinDistances();


    bool getTFs();

    void testAvoidance(ros::Duration period);

};



#endif
