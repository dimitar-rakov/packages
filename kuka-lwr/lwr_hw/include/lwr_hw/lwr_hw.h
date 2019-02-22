#ifndef LWR_HW__LWR_HW_H
#define LWR_HW__LWR_HW_H

#include <memory>
// ROS headers
#include <ros/node_handle.h>
#include <std_msgs/Duration.h>
#include <urdf/model.h>

// ROS controls
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <transmission_interface/transmission_info.h>
#include <transmission_interface/transmission_parser.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <control_toolbox/filters.h>

// KDL
#include <kdl/kdl.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp> //this to compute the gravity vector
#include <kdl_parser/kdl_parser.hpp>

//lwr specific hardware interfaces
#include <lwr_hw_interfaces/joint_impedance_state_interface.h>
#include <lwr_hw_interfaces/joint_impedance_command_interface.h>

namespace lwr_hw
{

class LWRHW : public hardware_interface::RobotHW
{
public:

  /// Configuration kuka lwr 4 has 7 joints
  const size_t n_joints_ = 7;

  /// Joint names
  std::vector<std::string> joint_names_;

  /// ControlStrategy
  /// JOINT_POSITION -> strategy 10 -> triggered with PoitionJointInterface
  /// CARTESIAN_IMPEDANCE -> strategy 20 (not implemented)
  /// JOINT_IMPEDANCE -> strategy 30 -> triggered with (ToDo) ImpedanceJointInterface
  /// JOINT_EFFORT -> strategy 90 with special configuration, triggered with EffortJointInterface
  enum ControlStrategy {JOINT_POSITION = 10, CARTESIAN_IMPEDANCE = 20, JOINT_IMPEDANCE = 30, JOINT_EFFORT = 90};

  /// Current control strategy
  ControlStrategy current_strategy_;

  /// Actual measured joints positions
  std::vector<double>joint_msr_position_;

  /// Previously measured joints positions
  std::vector<double>joint_msr_position_prev_;

  /// Actual measured joints velocity
  std::vector<double>joint_msr_velocity_;

  /// Previously measured joints velocity
  std::vector<double>joint_msr_velocity_prev_;

  /// Actual measured joints effort
  std::vector<double>joint_msr_effort_;

  /// Desired joints positions
  std::vector<double>joint_cmd_position_;

  /// Desired joints velocity
  /// NOTE: joint_velocity_cmd is not really to command the kuka arm in velocity,
  /// since it doesn't have an interface for that this is used to avoid speed
  /// limit error in the kuka controller by computing a fake velocity command
  /// using the received position command and the current position, without smoothing.
  std::vector<double>joint_cmd_velocity_;

  /// Desired joints effort
  std::vector<double>joint_cmd_effort_;

  /// Desired joints stiffness
  std::vector<double>joint_cmd_stiffness_;

  /// Desired joints damping
  std::vector<double>joint_cmd_damping_;

  /**
   * @brief LWRHW Default constructor
   */
  LWRHW() {}

  /**
   * @brief ~LWRHW Virtual default constructor
   */
  virtual ~LWRHW() {}

  /**
   * @brief Create allocates mmemory for a new arm
   * @param robot_namespace Robot namespace
   * @param urdf_string Urdf for robot description (xacro)
   */
  void create(std::string robot_namespace, std::string urdf_string);

  /**
   * @brief prepareSwitch Check if a controller from the start_list can be switched
   * @param start_list List with controller to be started
   * @param stop_list List with controller to be stopped
   */
  virtual bool prepareSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                             const std::list<hardware_interface::ControllerInfo> &stop_list) const;

  /**
   * @brief doSwitch Switch on controller from start_list and switch off controller from stop_list
   * @param start_list List with controller to be started
   * @param stop_list List with controller to be stopped
   */
  virtual void doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                        const std::list<hardware_interface::ControllerInfo> &stop_list);

  // These functions must be implemented depending on the outlet (Real FRI/FRIL, Gazebo, etc.)

  /**
   * @brief init Initialize a new lwr_hw
   * @param nh Node handle
   * @return True by success, false otherwise
   */
  virtual bool init(ros::NodeHandle &nh) = 0;

  /**
   * @brief read Read actual measurements for  joint handles
   * @param time Current ros time
   * @param period Last period used by integration and differentiation
   */
  virtual void read(ros::Time time, ros::Duration period) = 0;

  /**
   * @brief write Write command values for joint handles
   * @param time Current ros time
   * @param period Last period used by integration and differentiation
   */
  virtual void write(ros::Time time, ros::Duration period) = 0;

  /**
     * @brief Function to get the control strategy based on the list of controllers to be started/stopped
     *
     * @param start_list List with Controllers to be started
     * @param stop_list List with controller to be stopped
     * @param default_control_strategy Value to use as a default return value
     *
     * @return the output Control strategy found based on the lists
     */
  static ControlStrategy getNewControlStrategy(const std::list<hardware_interface::ControllerInfo> &start_list,
                                               const std::list<hardware_interface::ControllerInfo> &stop_list,
                                               ControlStrategy default_control_strategy = JOINT_POSITION);

  /**
   * @brief setControlStrategy Set a neu control strategy
   * @param strategy Strategy to be set
   */
  void setControlStrategy( ControlStrategy strategy){current_strategy_ = strategy;}

  /**
   * @brief getControlStrategy Return current strategy
   * @return curent strategy
   */
  ControlStrategy getControlStrategy(){ return current_strategy_;}

  /**
   * @brief enforceLimits Before write, you can use this function to enforce limits for all values
   * @param period Last period
   */
  void enforceLimits(const ros::Duration &period);

  /**
   * @brief reset All members variable are set to  their default values
   */
  void reset();

protected:
  /// Ros handle
  ros::NodeHandle nh_;

  /// Robot namespace
  std::string robot_namespace_;

  /// Joints names sufixes namespace
  const std::vector <std::string> join_names_sufixes_ {"_a1_joint",
                                                       "_a2_joint",
                                                       "_e1_joint",
                                                       "_a3_joint",
                                                       "_a4_joint",
                                                       "_a5_joint",
                                                       "_a6_joint"};

  /// Urdf model
  urdf::Model urdf_model_;

  /// KDL Solver for gravity effort
  std::unique_ptr <KDL::ChainDynParam> f_dyn_solver_;

  /// Gravity vector
  KDL::Vector gravity_vec_;

  /// Temporal joint position
  KDL::JntArray joint_position_kdl_;

  /// Temporal gravity effort
  KDL::JntArray gravity_effort_;

  /// KDL Robot chain
  std::unique_ptr <KDL::Chain> lwr_chain_;

  /// Joint lower limits
  std::vector<double> joint_lower_limits_;

  /// Joint upper limits
  std::vector<double> joint_upper_limits_;

  /// Joint effort limits have to be postive values
  std::vector<double> joint_effort_limits_;

  /// Joint velocity limits have to be postive values
  std::vector<double> joint_velocity_limits_;

  /// Joint acceleration limits have to be postive values
  std::vector<double> joint_acceleration_limits_;

  /// Joint jerk limits have to be postive values
  std::vector<double> joint_jerk_limits_;

  /// Urdf string with robot model
  std::string urdf_string_;

  /// Transmissions in this plugin's scope
  std::unique_ptr <std::vector<transmission_interface::TransmissionInfo>> transmissions_;

  /// Hardware interface for joint state measurements
  hardware_interface::JointStateInterface joint_state_interface_;

  /// Hardware interface for joint effort control
  hardware_interface::EffortJointInterface effort_interface_;

  /// Hardware interface for joint position control
  hardware_interface::PositionJointInterface position_interface_;

  /// Hardware interface for joint impedance state measurements
  hardware_interface::JointImpedanceStateInterface impedance_state_interface_;

  /// Hardware interface for joint impedance control
  hardware_interface::ImpedanceJointInterface impedance_interface_;

  /// Saturation interfaces joint position
  joint_limits_interface::PositionJointSaturationInterface pj_sat_interface_;

  /// Softlimit interfaces joint position
  joint_limits_interface::PositionJointSoftLimitsInterface pj_limits_interface_;

  /// Saturation interfaces joint velocity
  joint_limits_interface::VelocityJointSaturationInterface vj_sat_interface_;

  /// Softlimit interfaces joint velocity
  joint_limits_interface::VelocityJointSoftLimitsInterface vj_limits_interface_;

  /// Saturation interfaces joint effort
  joint_limits_interface::EffortJointSaturationInterface ej_sat_interface_;

  /// Softlimit interfaces joint effort
  joint_limits_interface::EffortJointSoftLimitsInterface ej_limits_interface_;


private:


  /**
   * @brief parseTransmissionsFromURDF Get Transmissions from the URDF
   * @param urdf_string Urdf_string of robot description (xacro)
   * @return True by success, otherwise false
   */
  bool parseTransmissionsFromURDF(const std::string& urdf_string);

  /**
   * @brief registerInterfaces Register all interfaces
   * @param urdf_model_ptr Pointer to a urdf model
   * @param transmissions Robot transmitions
   */
  void registerInterfaces(const urdf::Model *const urdf_model_ptr,
                          std::vector<transmission_interface::TransmissionInfo> transmissions);

  /**
   * @brief initKDLdescription Initialize all KDL members
   * @param urdf_model Urdf model
   * @return True by success, otherwise false
   */
  bool initKDLdescription(const urdf::Model *const urdf_model);

  /**
   * @brief registerJointLimits Helper function to register limit interfaces
   * @param joint_name Joint name
   * @param joint_handle_position Joint handle for position
   * @param joint_handle_effort Joint handle for effort
   * @param joint_handle_velocity Joint handle for velocity
   * @param urdf_model_ptr Pointer to a urdf model
   * @param joint_lower_limit Joint lower limits
   * @param joint_upper_limit Joint upper limits
   * @param velocity_limit Joint velocity limits
   * @param effort_limit Joint effort limits
   * @param acceleration_limit Joint acceleration limits
   * @param jerk_limit Joint jerk limits
   */
  void registerJointLimits(const std::string& joint_name,
                           const hardware_interface::JointHandle& joint_handle_position,
                           const hardware_interface::JointHandle& joint_handle_effort,
                           const hardware_interface::JointHandle& joint_handle_velocity,
                           const urdf::Model *const urdf_model_ptr,
                           double& joint_lower_limit, double & joint_upper_limit,
                           double& velocity_limit, double & effort_limit,
                           double& acceleration_limit, double& jerk_limit);



}; // class

} // namespace

#endif

