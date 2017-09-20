#ifndef VISUAL_FEATURES_EXTRACTOR_H
#define VISUAL_FEATURES_EXTRACTOR_H

//ROS headers
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/Pose.h>
#include <cv_bridge/cv_bridge.h>             //interface between ROS and OpenCV
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "tf/transform_datatypes.h"
#include <tf_conversions/tf_eigen.h>
#include "visual_features_extractor/VisFeature.h"
#include "visual_features_extractor/SetTF.h"
#include "visual_features_extractor/set_depth.h"
#include "sensor_msgs/JointState.h"


//OpenCV headers
#include <opencv2/core/core.hpp>


//Pthread Headers
#include <boost/thread/mutex.hpp>

//Eigen
#include <eigen3/Eigen/Dense>

//Aruco headers
#include <aruco/aruco.h>



class VisualFeaturesExtractor
{
public:

    VisualFeaturesExtractor();
    bool init(ros::NodeHandle &nh);
    void update(const ros::Time &time, const ros::Duration &period);

private:


    // Ros related variables
    ros::NodeHandle nh_;
    ros::ServiceServer srv_set_sim_features_tf_ ;
    ros::ServiceServer srv_set_des_features_tf_ ;
    ros::ServiceServer srv_set_des_template_ ;
    image_transport::ImageTransport it_;
    image_transport::Subscriber sub_img_transport_;
    ros::Publisher  pub_vis_features_;
    visual_features_extractor::VisFeature visualFeatureMsg_;
    sensor_msgs::CameraInfo cam_param_;

    boost::mutex  image_cb_mutex_;

    // tf related variables
    tf::TransformBroadcaster br_;
    tf::TransformListener lr_;
    std::vector <tf::StampedTransform> allTFwff_;
    std::vector <tf::Transform> allTFcdf_;
    tf::Transform TFwsf_;                           //simulated features wrt to world
    tf::Transform TFcdf_;                           //desired features wrt to camera
    tf::Transform TFwe_;                            //end effector wrt to world
    tf::Transform TFcw_;                            //world wrt to camera
    tf::Transform TFec_;                            //camera wrt to end effector

    // Parameters server variable
    bool using_sim_features_;
    bool using_extended_features_;
    bool using_symmetrical_features_;
    bool using_colored_blobs_;
    std::string points_topic_, raw_images_topic_, fixed_frame_, base_name_, camera_name_;
    std::vector <std::string> features_names_;
    double extended_features_var_;
    int num_features_;
    double contour_area_threshold_;

    // raw and central moments
    double xg_, yg_, m00_, m01_, m10_, m11_ , m20_, m02_, m12_;
    double m21_,m22_, m03_, m30_, m04_, m13_, m31_, m40_, mu11_, mu12_;
    double mu21_, mu02_, mu20_, mu03_, mu30_ ,mu02des_ , mu20des_;


    // variable related to specific moments
    double I1_, I2_, I3_, px_, py_, sx_, sy_, alpha_;
    double c1_, c2_, c3_, s1_, s2_, s3_, k_, an_, a_ ,a_des_;
    double t1_ ,t2_, P2_ ,P3_ , delta_, K_;

    //distance to feature wrt to camera origin
    double Z_, Z_des_;


    //// TEMP
    ///
    double quadrant_correction_ = 0.0;
    bool first_alpha_ = false;
    std::vector<double> data_;
    ros::Publisher  pub_all_data_;
    std_msgs::Float64MultiArray all_data_msg;

    ros::ServiceServer srv_start_features_rotation_ ;
    double msr_a_;
    bool enb_rotation_ = false;
    double msr_angle_, des_angle_;
    int idx_max_dist_;


    double theta3;

    ////////////////////////




    // Features related parameters
    Eigen::VectorXd x_;
    Eigen::VectorXd y_;
    Eigen::VectorXd s_msr_;
    Eigen::MatrixXd Lhat_msr_;
    Eigen::VectorXd s_des_;
    Eigen::MatrixXd Lhat_des_;


    // Data containers pointers
    cv_bridge::CvImagePtr cb_images_ptr_;
    cv_bridge::CvImagePtr in_images_ptr_;

    // destination image for a callback function
    cv::Mat  rect_image_, mask_image_, drawing_image_;

    // Aruco camera parameters
    aruco::CameraParameters aruco_cam_params_;

    // work features coordinates in image space
    std::vector< cv::Point2d>work_features_coord_;

    // simulated features coordinates in image space
    std::vector< cv::Point2d>sim_features_coord_;

    // desired features coordinates in image space
    std::vector< cv::Point2d>des_features_coord_;

    // hue limits for colored blobs
    std::vector <int> hue_min_limits_, hue_max_limits_;

    // ids for aruco markers
    std::vector <int> arucos_id;

    // sizes for aruco markers
    std::vector <double> arucos_size;

    // topics' data status : [-1] - not not received, [0] - delayed, status [1] - receive in time and data ok
    int image_status_;

    // Safety Timers for topics
    ros::Time safety_ton_image_;


    // service for setting a transform of the simulated features wrt to world
    bool setTfSimFeatureWrtWorld (visual_features_extractor::SetTF::Request &req, visual_features_extractor::SetTF::Response &res);

    // service for setting a transform of the desired features wrt to camera
    bool setTfDesFeatureWrtCamera(visual_features_extractor::SetTF::Request &req, visual_features_extractor::SetTF::Response &res);

    bool setDesiredTemplate(visual_features_extractor::set_depth::Request &req, visual_features_extractor::set_depth::Response &res);

    // function to get all transforms, needed for algorithms
    bool getTFs();

    // Get camera parameter from parameter server
    bool getCameraParameter(const ros::NodeHandle &nh, const std::string &camera_name, sensor_msgs::CameraInfo &dst_camera_param);

    // find contours of blobs
    void findBlobsContours(const cv::Mat &srs_image,std::vector<std::vector<cv::Point> >  &dst_contours, std::vector<std::string> &founded_features_names);

    // find aruco markers
    void findArucoMarkers(const cv::Mat &srs_image, std::vector<aruco::Marker>  &dst_markers, std::vector<std::string> &founded_features_names);

    // extract the center of features
    void showAll(const cv::Mat &srs_image, const std::vector<std::vector<cv::Point> > &src_contours, const std::vector<aruco::Marker>  &srs_markers,
                 const std::vector<cv::Point2d> &coord, const std::vector<std::string> &founded_features_names);

    // order the countours by size and take first numOfContours
    std::vector<std::vector<cv::Point> > getBestNContours(const std::vector<std::vector<cv::Point> > &srs_contours, int contour_area_treshold);

    // calculate image moments for given number of features and build matrices s, L,
    bool calcFeaturesParameters(const std::vector<cv::Point2d> &in_features_coord);

    // Callback function to images raw topic
    void imageCB(const sensor_msgs::ImageConstPtr& msg, cv_bridge::CvImagePtr *dst_image_ptr, ros::Time *safety_ton, int *image_status);

    //Calculate a central moment m_ij
    double calcSpatialMoment(const std::vector<cv::Point2d> &coord, const int &i, const int &j);

    //Calculate a central moment mu_ij
    double calcCentralMoment(const std::vector<cv::Point2d> &coord , const int &i, const int &j);

    //Calculation of classical feature vector and interaction matrix
    //based on features' image coordinates
    void calcSimple(Eigen::VectorXd &s, Eigen::MatrixXd &L);

    // Calculation of extended feature vector and interaction matrix
    // based on variant 1.0 features' moments for the case with asymmetrical features
    void calcExtendedAsymmetricalV1_0(Eigen::VectorXd &s, Eigen::MatrixXd &L_asym);

    // Calculation of extended feature vector and interaction matrix
    // based on variant 1.0 features' moments for the case with symmetrical features
    void calcExtendedSymmetricalV1_0(Eigen::VectorXd &s, Eigen::MatrixXd &L_sym);

    // Calculation of extended feature vector and interaction matrix
    // based on variant 1.1 features' moments for the case with asymmetrical features
    void calcExtendedAsymmetricalV1_1(Eigen::VectorXd &s, Eigen::MatrixXd &L_asym);

    // Calculation of extended feature vector and interaction matrix
    // based on variant 1.1 features' moments for the case with symmetrical features
    void calcExtendedSymmetricalV1_1(Eigen::VectorXd &s, Eigen::MatrixXd &L_sym);

    // Calculation of extended feature vector and interaction matrix
    // based on variant 1.2 features' moments for the case with asymmetrical features
    void calcExtendedAsymmetricalV1_2(Eigen::VectorXd &s, Eigen::MatrixXd &L_asym);

    // Calculation of extended feature vector and interaction matrix
    // based on variant 1.2 features' moments for the case with symmetrical features
    void calcExtendedSymmetricalV1_2(Eigen::VectorXd &s, Eigen::MatrixXd &L_sym);

    // Calculation of extended feature vector and interaction matrix
    // based on variant 1.3 features' moments for the case with asymmetrical features
    void calcExtendedAsymmetricalV1_3(Eigen::VectorXd &s, Eigen::MatrixXd &L_asym);

    // Calculation of extended feature vector and interaction matrix
    // based on variant 1.3 features' moments for the case with symmetrical features
    void calcExtendedSymmetricalV1_3(Eigen::VectorXd &s, Eigen::MatrixXd &L_sym);

    // Calculation of extended feature vector and interaction matrix
    // based on variant 1.4 features' moments for the case with asymmetrical features
    void calcExtendedAsymmetricalV1_4(Eigen::VectorXd &s, Eigen::MatrixXd &L_asym);

    // Calculation of extended feature vector and interaction matrix
    // based on variant 1.4 features' moments for the case with symmetrical features
    void calcExtendedSymmetricalV1_4(Eigen::VectorXd &s, Eigen::MatrixXd &L_sym);


    // Calculate all simulated features transforms wrt to world (Euclidean coordinates)
    void simVisualFeaturesTFs();

    // Calculate all desired features transforms wrt to camera (pixel coordinates)
    void desVisualFeaturesTFs();

    // Calculate all features vectors
    void calcFeaturesImageCoord();


   bool startFeaturesRotation(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res);


};

#endif // VIS_FEATURE_EXTRACT_H
