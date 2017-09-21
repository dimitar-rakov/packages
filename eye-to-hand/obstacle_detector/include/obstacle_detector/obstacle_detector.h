#ifndef OBSTACLEDETECTOR_H
#define OBSTACLEDETECTOR_H

//ROS headers
#include "ros/ros.h"
#include "pcl_ros/point_cloud.h"
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "obstacle_detector/SetObstacleTrajectory.h"

// TF
#include <tf/transform_listener.h>
#include "tf/transform_datatypes.h"


//Standard Headers
#include <stdlib.h>

// Boost
#include <boost/thread/mutex.hpp>


//PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>



static const bool USE_PCL_CB = true;

class ObstacleDetector
{
public:
    // variables

    // methods
    ObstacleDetector();

    // parameter for feature extraction
    bool init(ros::NodeHandle &nh);

    // all calculations
    void update(const ros::Time &time, const ros::Duration &period);

    // all publisher are inside
    void publish();



private:
    /// variables

    // Ros related variables
    ros::NodeHandle nh_;
    ros::Subscriber sub_point_cloud_;
    ros::ServiceServer srv_set_obstacle_trajectory_ ;
    ros::Publisher pub_output_cloud_;
    ros::Publisher pub_markers_obstacle_objects_;
    ros::Publisher pub_markers_filter_env_objects_;
    ros::Publisher pub_markers_filter_robot_objects_;
    ros::Publisher pub_markers_filter_robot_objects_fixed_;

    //std::mutex image_mutex_, points_mutex;
    boost::mutex   points_cb_mutex_;

    double radius_sphere_obstacle, radius_sphere_robot_body_; //in meters
    tf::TransformListener lr_;
    std::vector<tf::Transform> TFs_, TFs_fixed_;              //all joints TFs

    visualization_msgs::MarkerArray obstacle_objects_;
    visualization_msgs::MarkerArray filter_env_objects_;
    visualization_msgs::MarkerArray filter_robot_objects_;
    visualization_msgs::MarkerArray filter_robot_objects_fixed_;

    sensor_msgs::PointCloud2::Ptr cb_ros_cloud_ptr_;
    sensor_msgs::PointCloud2::Ptr filtered_ros_cloud_ptr_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_ptr_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cb_cloud_ptr_;


    // Parameters server variable
    bool sim_obstacles_enb_;
    std::string points_topic_;
    std::string fixed_frame_;
    std::string base_name_;
    std::vector<std::string> tf_names_;
    double obs_octree_resolution_;
    int min_voxel_points_;


    // flags for topics data. status -1 - not not received, status 0 - delayed,
    // status 1 - receive in time and data ok, status 2 - receive in time and not valid data,
    int points_status_;

    // Safety Timers for topics
    ros::Time safety_ton_points_;

    // Moving obstacle parameteres
    tf::Vector3 obst_start_position_;
    tf::Vector3 obst_end_position_;
    double obs_start_time_;
    double obst_goal_time_;
    double obst_curr_distance_;
    bool new_move_;

    /// methods

    bool setObstacleTrajectory(obstacle_detector::SetObstacleTrajectory::Request &req, obstacle_detector::SetObstacleTrajectory::Response &res);

    void pclPointcloudCB (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud_msg, pcl::PointCloud<pcl::PointXYZ>::Ptr *ptr_dst_cloud, ros::Time *safety_ton, int *points_status);

    void rosPointcloudCB(const sensor_msgs::PointCloud2ConstPtr& msg, sensor_msgs::PointCloud2Ptr *ptr_dst_cloud, ros::Time *safety_ton, int *points_status);

    bool getTFs();

    void buildRobotBodyFromSpheres();

    void getFilterObjectsParameters(XmlRpc::XmlRpcValue &obj, visualization_msgs::MarkerArray &filter_objects, const std::string &ns);

    void filterBoxOut(const visualization_msgs::Marker &filter_objects, const pcl::PointCloud<pcl::PointXYZ>::Ptr &ptr_in_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &ptr_filtered_cloud);

    void filterSphereOut(const visualization_msgs::Marker &filter_objects, const pcl::PointCloud<pcl::PointXYZ>::Ptr &ptr_in_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &ptr_filtered_cloud);

    void simMovingObstacle(const ros::Duration &period);

    void detectObstaclesAsBox();

    void detectOctreeVoxels();
};

#endif // OBSTACLEDETECTOR_H
