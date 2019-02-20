#ifndef LWR_TRAJ_GENERATOR_H
#define LWR_TRAJ_GENERATOR_H

//Standard Headers
#include <stdlib.h>
#include <mutex>

//ROS headers
#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>

class LwrTrajGenerator
{
public:

  /**
      * @brief LwrTrajGenerator Default constructor
      */
  LwrTrajGenerator();

  /**
       * @brief init Initilializing of LwrTrajGenerator. It has to be called only once.
       * @param nh node handle
       * @return true if everithing pass normaly, false otherwise
       */
  void init (ros::NodeHandle &nh);

  /**
       * @brief update Periodicaly called. All calculations related to this class are done within.
       * @param time Time from start
       * @param period Last update period
       */
  void update(const ros::Time& time, const ros::Duration& period);

  /**
     * @brief command Callback function for command different action
     * @param msg desired position e.g. start, stop, home up
     */
  void command(const std_msgs::String &msg);

  /**
   * @brief getJointPosition Callback for measured joint positions
   * @param msg Joint state
   */
  void getJointPosition(const sensor_msgs::JointState &msg);

private:
  /**
     * @brief startTrajectroies Start robot joint trajectories
     */
  void startTrajectroies();

  /**
     * @brief stopTrajectroies Stop current robot joint trajectories
     */
  void stopTrajectroies();

  /**
     * @brief goToPosition
     * @param goal_pos Desired roboter joint position
     */
  void goToPosition(const std::vector<double> &goal_pos);

  /// Node handle
  ros::NodeHandle nh_;

  /// Publisher posture
  ros::Publisher  pub_posture_;

  /// Subscriber for command callback
  ros::Subscriber sub_command_;

  /// Subscriber for joint state
  ros::Subscriber sub_joint_state_;

  /// All trajectories container
  std::vector<std::vector<double>> all_traj;

  /// Measured joint position
  std::vector<double> joint_msr_pos_;

  /// Homing joint position
  std::vector<double>  home_pos_;

  /// Command position work
  std_msgs::Float64MultiArray::Ptr cmd_msg_ptr_;

  /// Work flag
  bool flag_work_;

  /// Elapsed time since last trajectory start
  ros::Duration elapsed_time_;

};

#endif // LWR_TRAJ_GENERATOR_H
