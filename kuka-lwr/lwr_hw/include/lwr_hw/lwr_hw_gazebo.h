#ifndef LWR_HW_GAZEBO_H
#define LWR_HW_GAZEBO_H

// ROS
#include <angles/angles.h>
// #include <pluginlib/class_list_macros.h>

// Gazebo hook
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>

// lwr hw definition
#include "lwr_hw/lwr_hw.h"

namespace lwr_hw {


class LWRHWGazebo : public LWRHW
{
public:

  LWRHWGazebo();
  ~LWRHWGazebo() final;

  /**
   * @brief init Initialize a new lwr_hw
   * @param nh Node handle
   * @return True by success, false otherwise
   */
  bool init(ros::NodeHandle &nh) final;

  /**
   * @brief write Write command values for joint handles
   * @param time Current ros time
   * @param period Last period used by integration and differentiation
   */
  void read(ros::Time time, ros::Duration period) final;

  /**
   * @brief write Write command values for joint handles
   * @param time Current ros time
   * @param period Last period used by integration and differentiation
   */
  void write(ros::Time time, ros::Duration period) final;

  /**
   * @brief setParentModel Set parent model in Gazebo
   * @param parent_model_ptr
   */
  void setParentModel(gazebo::physics::ModelPtr parent_model_ptr);


private:

  // workaround: simple P controller to overcome the ofsets caused by physical properties of the gazebo model
  // ToDo for improvement to move this simple controllers in a separate thread with fixed sampling
  std::vector<double> joint_cmd_position_add_;
  std::vector<double> joint_cmd_effort_add_;    //Because of specific of gazebo 2 does not work, therefore not used. ToDo test in gazebo > 4

  // Robot joints in gazebo
  std::vector<gazebo::physics::JointPtr> sim_joints_;

  /// Gazebo physics model
  gazebo::physics::ModelPtr parent_model_ptr;

  /// Indicate whatever the physics model was set
  bool parent_set_ = false;


};


} // namespace


#endif // LWR_HW_GAZEBO_H
