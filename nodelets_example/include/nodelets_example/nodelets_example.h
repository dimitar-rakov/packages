#ifndef NODELETS_EXAMPLE_H
#define NODELETS_EXAMPLE_H

//ROS headers
#include "ros/ros.h"
#include <visualization_msgs/MarkerArray.h>
#include <mutex>

namespace examples
{

class NodeletsExample
{
public:

  /**
   * @brief NodeletsExample Default constructor
   */
  NodeletsExample();

  /**
     * @brief init Initilializing of NodeletsExample. It has to be called only once.
     * @param nh node handle
     * @return true if everithing pass normaly, false otherwise
     */
  bool init(ros::NodeHandle &nh);

  /**
     * @brief update Periodicaly called. All calculations related to this class are done within.
     * @param time Time from start
     * @param period Last update period
     */
  void update(const ros::Time& time, const ros::Duration& period);



private:

  /// Node handle
  ros::NodeHandle nh_;

  /// Publisher markers
  ros::Publisher pub_markers_;

  /// Subscriber markers
  ros::Subscriber sub_markers_;

  /// Topic name for marker's publisher
  std::string pub_markers_topic_;

  /// Topic name for marker's publisher
  std::string sub_markers_topic_;

  /// Node base name
  std::string base_name_;

  /// enable marker publisher
  bool enb_publishing_;

  /// Mutex for markers callback
  std::mutex markers_cb_mutex_;

  /// Marker array const pointer used in update function
  visualization_msgs::MarkerArray::ConstPtr in_markers_ptr_;

  /// Marker array const pointer used in callbacks function
  visualization_msgs::MarkerArray::ConstPtr cb_markers_ptr_;

  /**
   * @brief markersCB Markers calback function
   * @param msg Input Marker array pointer
   */
  void markersCB(const visualization_msgs::MarkerArray::ConstPtr &msg);

};
} // end of namespace examples

#endif // NODELETS_EXAMPLE_H
