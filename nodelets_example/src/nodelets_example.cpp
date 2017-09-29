#include "nodelets_example/nodelets_example.h"
#include <boost/thread/lock_guard.hpp>

namespace examples
{


NodeletsExample::NodeletsExample() { }

bool NodeletsExample::init(ros::NodeHandle &nh)
{
  // Ros node handle for the class
  nh_ = nh;
  cb_markers_ptr_.reset (new visualization_msgs::MarkerArray ());
  in_markers_ptr_.reset (new visualization_msgs::MarkerArray ());

  // Get base_name from parameter server
  if (!nh_.getParam("base_name", base_name_)){
    nh_.param("base_name", base_name_, std::string ("nodelets_example"));
    ROS_WARN("Parameter base_name was not found. Default name is used: %s ", base_name_.c_str());
  }

  // Get pub_markers_topic  from parameter server
  if (!nh_.getParam("pub_markers_topic", pub_markers_topic_)){
    nh_.param("pub_markers_topic", pub_markers_topic_, std::string ("/markers_topic"));
    ROS_WARN("Parameter pub_markers_topic was not found. Default topic's name is used: %s ", pub_markers_topic_.c_str());
  }

  // Get sub_markers_topic  from parameter server
  if (!nh_.getParam("sub_markers_topic", sub_markers_topic_)){
    nh_.param("sub_markers_topic", sub_markers_topic_, std::string ("/markers_topic"));
    ROS_WARN("Parameter sub_markers_topic was not found. Default topic's name is used: %s ", sub_markers_topic_.c_str());
  }


  // Get enb_obstacle_avoidance  from parameter server
  if (!nh_.getParam("enb_publishing", enb_publishing_)){
    nh_.param("enb_publishing", enb_publishing_, false);
    ROS_WARN("Parameter enb_publishing was not found. Default value is used: false");
  }

  pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>(pub_markers_topic_, 10 );
  sub_markers_ = nh_.subscribe(sub_markers_topic_, 10, &NodeletsExample::markersCB, this);

  ROS_INFO ("NodeletsExample with a name %s is initialized", base_name_.c_str());
  return true;

}

void NodeletsExample::update(const ros::Time& time, const ros::Duration& period){
  // Safety mutex
  {
    boost::lock_guard<boost::mutex> guard(markers_cb_mutex_);
    in_markers_ptr_ = cb_markers_ptr_;
  }

  /* Do some work with in_markers_ptr_   */


  // just for test delays
  if (enb_publishing_){
    visualization_msgs::MarkerArray::Ptr markers_ptr(new visualization_msgs::MarkerArray ()) ;
    for (size_t i = 0; i < 10000; i++){

      // parametrize the marker with founded coeficients
      visualization_msgs::Marker marker;
      marker.header.frame_id = "world";
      marker.header.stamp = ros::Time();
      marker.ns = "obstacles";
      marker.id = 3002;
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.scale.x =  0.1;
      marker.scale.y =  0.1;
      marker.scale.z =  0.1;
      marker.pose.position.x= 0;
      marker.pose.position.y= 0;
      marker.pose.position.z= i/10000;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.color.a = 0.5;
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.text = std::string("KEEP");
      marker.lifetime = ros::Duration(0.5);
      markers_ptr->markers.push_back(marker);
    }
    markers_ptr->markers[0].header.stamp = ros::Time::now();
    ros::Time tic = ros::Time::now();
    pub_markers_.publish(markers_ptr);
    ROS_INFO ("Delay for publishing %lf " ,(ros::Time::now() - tic).toSec());
  }
}

void NodeletsExample::markersCB(const visualization_msgs::MarkerArray::ConstPtr &msg){
  boost::lock_guard<boost::mutex> guard(markers_cb_mutex_);
  ROS_INFO ("Calback in obstacleObjectsCB from %s received. Delay %lf " ,
            base_name_.c_str(), (ros::Time::now() - msg->markers[0].header.stamp).toSec());
  cb_markers_ptr_ = boost::const_pointer_cast<visualization_msgs::MarkerArray>(msg);

}
} // end of namespace examples
