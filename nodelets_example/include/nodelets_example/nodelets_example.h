#ifndef NODELETS_EXAMPLE_H
#define NODELETS_EXAMPLE_H

//ROS headers
#include "ros/ros.h"
#include <visualization_msgs/MarkerArray.h>
#include <boost/thread/mutex.hpp>

namespace examples
{

class NodeletsExample
{
public:
  NodeletsExample();
  // initilize the node handle
  bool init(ros::NodeHandle &nh);

  //periodicaly called inside of EXAMPLE_PACKAGE. All publisher are there
  void update(const ros::Time& time, const ros::Duration& period);



private:
  ros::NodeHandle nh_;
  ros::Publisher pub_markers_;
  ros::Subscriber sub_markers_;

  //parameter server vars
  std::string pub_markers_topic_;
  std::string sub_markers_topic_;
  std::string base_name_;
  bool enb_publishing_;

  // thread safety
  boost::mutex   markers_cb_mutex_;
  visualization_msgs::MarkerArray::Ptr in_markers_ptr_;
  visualization_msgs::MarkerArray::Ptr cb_markers_ptr_;

  void markersCB(const visualization_msgs::MarkerArray::ConstPtr &msg);

};
} // end of namespace examples

#endif // NODELETS_EXAMPLE_H
