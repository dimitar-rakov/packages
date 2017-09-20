#ifndef KINECT_FUSION_H
#define KINECT_FUSION_H

//ROS headers
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


// kinect_fusion  specific msg and srv
#include "kinect_fusion/SetTask.h"

// Open CV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <aruco/aruco.h>

//Standard Headers
#include <stdlib.h>
#include <boost/thread/mutex.hpp>



#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

static const int QUEUE_SIZE =3;


class KinectFusion
{
public:
    typedef sensor_msgs::PointCloud2 PC2;
    typedef message_filters::sync_policies::ApproximateTime<PC2, PC2, PC2, PC2, PC2, PC2>sync_policy6;
    KinectFusion();
    // initilize the node handle
    bool init(ros::NodeHandle &nh);

    //periodicaly called inside of kinect_fusion_node. All publisher are there
    void update(const ros::Time& time, const ros::Duration& period);

private:

    ros::NodeHandle nh_;

    // subscribers to the points
    std::vector<ros::Subscriber> subs_points_;

    // subscribers to the cameras info
    std::vector<ros::Subscriber> subs_cam_info_;

    std::vector<boost::shared_ptr<message_filters::Subscriber<PC2> > > msg_filters_;
    boost::shared_ptr<message_filters::Synchronizer<sync_policy6> > ts_a6_;


    ros::Publisher pub_points;
    ros::Publisher pub_points1;
    ros::Publisher pub_points2;
    std::vector< tf::Transform >  TFs_w_c;
    std::vector<tf::Transform> TFs_a_c_;
    tf::TransformBroadcaster br_;
    tf::TransformListener lr_;

    // subscribers to the raw images
    std::vector<image_transport::Subscriber> subs_image_raw_;
    std::vector<image_transport::ImageTransport> images_tran_;

    // Data containers pointers
    std::vector<cv_bridge::CvImagePtr> cb_images_ptr_;
    std::vector<cv_bridge::CvImagePtr> in_images_ptr_;

    std::vector<sensor_msgs::CameraInfo::Ptr> cb_cam_info_ptr_;
    std::vector<sensor_msgs::CameraInfo::Ptr> in_cam_info_ptr_;

    std::vector<sensor_msgs::PointCloud2::Ptr> cb_clouds_ptr_;
    std::vector<sensor_msgs::PointCloud2::Ptr> in_clouds_ptr_;
    std::vector<sensor_msgs::PointCloud2::Ptr> transf_clouds_ptr_;
    sensor_msgs::PointCloud2::Ptr fused_cloud_ptr_;

    // Paramer server variables
    std::string base_name_;
    std::vector<std::string> raw_images_topics_, point_topics_, cam_info_topics_;
    bool using_aruco_ ;
    double aruco_marker_size_;

    //Aruco related variables
    int aruco_marker_id_; //not used

    // flags for topics data. status -1 - not not received, status 0 - delayed,
    // status 1 - receive in time and data ok, status 2 - receive in time and not valid data,
    std::vector<int> image_status_, cam_info_status_ ;
    int points_status_;

    // Safety Timers for topics
    std::vector<ros::Time> safety_tons_images_, safety_tons_cam_info_;
    ros::Time safety_tons_points_;

    boost::mutex image_cb_mutex_, sync_cb_mutex_, cam_info_cb_mutex_;

    // Extract aruco marker
    void markerDetect(const cv:: Mat& srs_image, const sensor_msgs::CameraInfoPtr &cam_info_ptr, tf::Transform &dstTF, int marker_id, double marker_size, std::string windows_name );

    // Callback function to images raw topic
    void imageCB(const sensor_msgs::ImageConstPtr& msg, cv_bridge::CvImagePtr *dst_image_ptr, ros::Time *safety_ton, int *image_status);

    // Callback function to camera info
    void cameraInfoCB(const sensor_msgs::CameraInfoConstPtr& msg, sensor_msgs::CameraInfoPtr* dst_cam_info_ptr, ros::Time *dst_safety_ton, int *dst_status);

    // Synchronized callback function to all point raw topic
    void syncPointcloudsCB( const sensor_msgs::PointCloud2ConstPtr& msg1, const sensor_msgs::PointCloud2ConstPtr& msg2,
                          const sensor_msgs::PointCloud2ConstPtr& msg3, const sensor_msgs::PointCloud2ConstPtr& msg4,
                          const sensor_msgs::PointCloud2ConstPtr& msg5, const sensor_msgs::PointCloud2ConstPtr& msg6);

    void pointcloudsFusion(std::vector<sensor_msgs::PointCloud2::Ptr>  in_clouds_ptr);
};

#endif // KINECT_FUSION_H
