#include "kinect_fusion/kinect_fusion.h"
#include "boost/bind/bind.hpp"
#include "pcl_ros/transforms.h"
#include "opencv2/calib3d/calib3d.hpp"
#include <boost/lexical_cast.hpp>

#include <pcl/filters/filter.h>

KinectFusion::KinectFusion(){ }

bool KinectFusion::init(ros::NodeHandle &nh)
{
  // Ros node handle for the class
  nh_ = nh;
  marker_size_ = 0.186;
  marker_size_ = 0.265;
  marker_id_= 66;

  // Get base_name from parameter server
  if (!nh_.getParam("base_name", base_name_)){
    nh_.param("base_name", base_name_, std::string ("kinect_fusion"));
    ROS_WARN("Parameter base_name was not found. Default name is used: %s ", base_name_.c_str());
  }

  // Get raw image topics from parameter server
  if (!nh_.getParam("raw_images_topics", raw_images_topics_)){
    nh_.param("raw_images_topics", raw_images_topics_, std::vector<std::string> {"/kinect2_k1/qhd/image_color", "/kinect2_k2/qhd/image_color"});
    ROS_WARN("Parameter raw_images_topics was not found.");
    for (std::string topic : raw_images_topics_){ ROS_WARN("Default topic's name is used: %s ", topic.c_str());}
  }

  // Get camera info topics from parameter server
  if (!nh_.getParam("cameras_info_topics", cam_info_topics_)){
    nh_.param("cameras_info_topics", cam_info_topics_, std::vector<std::string> {"/kinect2_k1/hd/camera_info", "/kinect2_k2/hd/camera_info"});
    ROS_WARN("Parameter cameras_info_topics was not found.");
    for (std::string topic : cam_info_topics_){ ROS_WARN("Default topic's name is used: %s ", topic.c_str());}
  }

  if (raw_images_topics_.size() !=cam_info_topics_.size()){
    ROS_WARN("Sizes of raw_images_topics and cameras_info_topics are different. Aruco detection cannot be used");
  }

  // Get point topics from parameter server
  if (!nh_.getParam("points_topics", point_topics_)){
    nh_.param("points_topics", point_topics_, std::vector<std::string> {"/kinect2_k1/sd/points", "/kinect2_k2/sd/points"});
    ROS_WARN("Parameter points_topics was not found.");
    for (std::string topic : point_topics_){ ROS_WARN("Default topic's name is used: %s ", topic.c_str());}
  }

  for (std::string topic : point_topics_){ ROS_WARN("Default topic's name is used: %s ", topic.c_str());}

  if (point_topics_.size () > 6){
    ROS_ERROR ("More than 6 topics are found!");
    return false;
  }



  TFs_w_c.resize(point_topics_.size (), tf::Transform::getIdentity());
  TFs_a_c_.resize(cam_info_topics_.size(), tf::Transform::getIdentity());

  /// TODO remove all ///////////////////////////////////////////////////////////////
  tf::Transform TFa_w , TFa_rgb1, TFa_rgb2, TFrgb1_a, TFrgb2_a, TFrgb_ir1, TFrgb_ir2;
  tf::Quaternion q;
  tf::Vector3 v;


  // Aruco wrt Cameras
  TFrgb1_a.setOrigin(tf::Vector3(-0.627018, -1.067080, 3.404692));
  TFrgb1_a.setRotation(tf::Quaternion(0.087088, 0.927167, -0.230036,-0.282597));
  TFrgb2_a.setOrigin(tf::Vector3(0.336732, -0.992832, 3.265763));
  TFrgb2_a.setRotation(tf::Quaternion(-0.073189, 0.931533, -0.261369,0.242027));

  // Cameras wrt Aruco
  TFa_rgb1 = TFrgb1_a.inverse();
  TFa_rgb2 = TFrgb2_a.inverse();


  //Cameras ir wrt rgb
  TFrgb_ir1.setOrigin( tf::Vector3(0.0569519848386, -0.00359629842383, -0.00612798476349));   //Manualy set 0.0569519848386 instead of -0.0569519848386
  TFrgb_ir1.setRotation(tf::Quaternion(0.00234451457864, -0.00436009068321, -0.000514417778053, 0.999987614041));
  TFrgb_ir2.setOrigin( tf::Vector3(-0.0559727167749, -0.00536732469295, -0.0105464068071));
  TFrgb_ir2.setRotation(tf::Quaternion(0.00418506953665, -0.00255933638988, -0.00220228654428, 0.999985542358));

  //World wrt to aruco
  v = tf::Vector3(-0.15, 0.0, -0.75);
  q.setRPY(M_PI_2, 0, M_PI);
  TFa_w.setIdentity();
  TFa_w.setOrigin(v);
  TFa_w.setRotation(q);

  TFs_w_c[0] = (TFrgb_ir1*TFrgb1_a).inverse();
  TFs_w_c[1] = (TFrgb_ir2*TFrgb2_a).inverse();

  TFs_w_c[0] = (TFrgb_ir1*TFrgb1_a*TFa_w).inverse();
  TFs_w_c[1] = (TFrgb_ir2*TFrgb2_a*TFa_w).inverse();


  v = TFs_w_c[0].getOrigin();
  q = TFs_w_c[0].getRotation();
  ROS_INFO ("TFir1_w: %lf %lf %lf %lf %lf %lf %lf", v.x(), v.y(), v.z(), q.x(), q.y(), q.z(),q.w());


  v = TFs_w_c[1].getOrigin();
  q = TFs_w_c[1].getRotation();
  ROS_INFO ("TFir2_w: %lf %lf %lf %lf %lf %lf %lf", v.x(), v.y(), v.z(), q.x(), q.y(), q.z(),q.w());

  /////////////////////////////////////////////////////////////////////////////////////////////////////////


  images_tran_.resize(raw_images_topics_.size(), image_transport::ImageTransport(nh_));
  subs_image_raw_.resize(raw_images_topics_.size());
  cb_images_ptr_.resize(raw_images_topics_.size());
  in_images_ptr_.resize(raw_images_topics_.size());
  image_status_.resize(raw_images_topics_.size(), -1);
  safety_tons_images_.resize(raw_images_topics_.size());

  subs_cam_info_.resize(cam_info_topics_.size());
  cb_cam_info_ptr_.resize(cam_info_topics_.size());
  in_cam_info_ptr_.resize(cam_info_topics_.size());
  cam_info_status_.resize(cam_info_topics_.size(), -1);
  safety_tons_cam_info_.resize(cam_info_topics_.size());

  subs_points_.resize(point_topics_.size());
  cb_clouds_ptr_.resize(point_topics_.size());
  in_clouds_ptr_.resize(point_topics_.size());
  transf_clouds_ptr_.resize(point_topics_.size());
  points_status_= -1;
  safety_tons_points_ = ros::Time::now();
  msg_filters_.resize(point_topics_.size());
  fused_cloud_ptr_.reset(new sensor_msgs::PointCloud2());


  // Initialize  images subscribers and images containers pointers
  for (size_t i = 0; i <  raw_images_topics_.size(); i++){
    cb_images_ptr_[i].reset (new cv_bridge::CvImage());
    in_images_ptr_[i].reset (new cv_bridge::CvImage());
    if (USE_ARUCO)
      subs_image_raw_[i] = images_tran_[i].subscribe(raw_images_topics_[i], 1, boost::bind(&KinectFusion::imageCB, this, _1, &cb_images_ptr_[i], &safety_tons_images_[i], &image_status_[i]));
  }

  // Initialize  camera info subsribers and cam_info containers pointers
  for (size_t i = 0; i <  cam_info_topics_.size(); i++){
    cb_cam_info_ptr_[i].reset (new sensor_msgs::CameraInfo());
    in_cam_info_ptr_[i].reset (new sensor_msgs::CameraInfo());
    if (USE_ARUCO)
      subs_cam_info_[i] = nh_.subscribe<sensor_msgs::CameraInfo>(cam_info_topics_[i], 1, boost::bind(&KinectFusion::cameraInfoCB, this, _1, &cb_cam_info_ptr_[i], &safety_tons_cam_info_[i], &cam_info_status_[i]));
  }

  // Initialize filters for points topics and cointeiners for transformed pointers
  for (size_t i = 0; i < point_topics_.size (); i++){
    msg_filters_[i].reset (new message_filters::Subscriber<PC2> ());
    msg_filters_[i]->subscribe (nh_, point_topics_[i], QUEUE_SIZE);
    transf_clouds_ptr_[i].reset(new sensor_msgs::PointCloud2());
    in_clouds_ptr_[i].reset(new sensor_msgs::PointCloud2());
    cb_clouds_ptr_[i].reset(new sensor_msgs::PointCloud2());
  }


  ts_a6_.reset (new message_filters::Synchronizer<sync_policy6> (QUEUE_SIZE));
  // Use the first filter as default, when the number of topics is less than 6
  switch (point_topics_.size()) {
  case 2:
    ts_a6_->connectInput (*msg_filters_[0], *msg_filters_[1], *msg_filters_[0], *msg_filters_[0], *msg_filters_[0], *msg_filters_[0]);
    break;
  case 3:
    ts_a6_->connectInput (*msg_filters_[0], *msg_filters_[1], *msg_filters_[2], *msg_filters_[0], *msg_filters_[0], *msg_filters_[0]);
    break;
  case 4:
    ts_a6_->connectInput (*msg_filters_[0], *msg_filters_[1], *msg_filters_[2], *msg_filters_[3], *msg_filters_[0], *msg_filters_[0]);
    break;
  case 5:
    ts_a6_->connectInput (*msg_filters_[0], *msg_filters_[1], *msg_filters_[2], *msg_filters_[3], *msg_filters_[4], *msg_filters_[0]);
    break;
  case 6:
    ts_a6_->connectInput (*msg_filters_[0], *msg_filters_[1], *msg_filters_[2], *msg_filters_[3], *msg_filters_[4], *msg_filters_[5]);
    break;
  default:
    ROS_ERROR ("Inappropriate size of the toipics!");
    break;
  }

  ts_a6_->registerCallback (boost::bind (&KinectFusion::syncPointcloudsCB, this, _1, _2, _3, _4, _5, _6));

  pub_points = nh_.advertise<sensor_msgs::PointCloud2> (nh_.getNamespace()+"/points", 1);
  pub_points1 = nh_.advertise<sensor_msgs::PointCloud2> (nh_.getNamespace()+"/points1", 1);
  pub_points2 = nh_.advertise<sensor_msgs::PointCloud2> (nh_.getNamespace()+"/points2", 1);
  srv_task_number_ = nh_.advertiseService("/set_task_number", &KinectFusion::setTaskNumber, this);

  ROS_INFO ("KinectFusion with name %s is initialized", base_name_.c_str());
  return true;

}

void KinectFusion::update(const ros::Time& time, const ros::Duration& period){

  if (USE_ARUCO){
    // Safety timers and mutexes images
    boost::lock_guard<boost::mutex> guard(image_cb_mutex_);
    for (size_t i = 0; i< image_status_.size(); i++){
      if ((ros::Time::now()- safety_tons_images_[i]).toSec()< 2.0 && image_status_[i]> -1){ image_status_[i]; }
      else if ((ros::Time::now()- safety_tons_images_[i]).toSec()> 2.0 && image_status_[i]> -1){ image_status_[i]= 0; }
      if ( image_status_[i] == 0) ROS_WARN("Camera's topic[%ld] is not longer available", i);
      else if (image_status_[i] == -1) ROS_WARN_THROTTLE(5, "Waiting for image's topic");
      in_images_ptr_[i] = cb_images_ptr_[i];
    }
  }

  if (USE_ARUCO){
    // Safety timers and mutexes camera info
    boost::lock_guard<boost::mutex> guard(cam_info_cb_mutex_);
    for (size_t i = 0; i< cam_info_status_.size(); i++){
      if ((ros::Time::now()- safety_tons_cam_info_[i]).toSec()< 2.0 && cam_info_status_[i]> -1){ cam_info_status_[i]; }
      else if ((ros::Time::now()- safety_tons_cam_info_[i]).toSec()> 2.0 && cam_info_status_[i]> -1){ cam_info_status_[i]= 0; }
      if ( cam_info_status_[i] == 0) ROS_WARN("Camera's info topic[%ld] is not longer available", i);
      else if (cam_info_status_[i] == -1) ROS_WARN_THROTTLE(5, "Waiting for camera info's topic");
      in_cam_info_ptr_[i] = cb_cam_info_ptr_[i];
    }
  }

  {
    // Safety timer and mutex synchronizer
    boost::lock_guard<boost::mutex> guard(sync_cb_mutex_);
    if ((ros::Time::now()- safety_tons_points_).toSec()< 2.0 && points_status_> -1){ points_status_= 1; }
    else if ((ros::Time::now()- safety_tons_points_).toSec()> 2.0 && points_status_> -1){ points_status_= 0; }
    if (points_status_ == 0 ) ROS_WARN("Synchronizing is not longer available" );
    else if (points_status_ == -1) ROS_WARN_THROTTLE(5, "Waiting for synchronizing");
    in_clouds_ptr_ = std::vector<sensor_msgs::PointCloud2::Ptr>( cb_clouds_ptr_);
  }

  /// DEBUG
  for (size_t i=0; i < in_images_ptr_.size(); i++){
    if(!in_images_ptr_[i]->image.empty() && !in_cam_info_ptr_[i]->D.empty()){
      markerDetect(in_images_ptr_[i]->image, in_cam_info_ptr_[i], TFs_a_c_[i], marker_id_, marker_size_, std::string("Sensor ") + boost::lexical_cast<std::string>(i) );
    }
  }
}

void KinectFusion::imageCB(const sensor_msgs::ImageConstPtr& msg, cv_bridge::CvImagePtr *dst_image_ptr, ros::Time *safety_ton, int *image_status){
  boost::lock_guard<boost::mutex> guard(image_cb_mutex_);
  *safety_ton = ros::Time::now();
  try{
    // transform ROS image into OpenCV image
    *dst_image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    *image_status = 1;
  }
  catch (cv_bridge::Exception& e){
    // throw an error msg. if conversion fails
    ROS_ERROR("cv_bridge exception: %s", e.what());
    *image_status = 2;
    return;
  }
}

void KinectFusion::cameraInfoCB(const sensor_msgs::CameraInfoConstPtr& msg, sensor_msgs::CameraInfoPtr *dst_cam_info_ptr, ros::Time *dst_safety_ton, int *dst_status ){
  boost::lock_guard<boost::mutex> guard(cam_info_cb_mutex_);
  *dst_cam_info_ptr = boost::const_pointer_cast<sensor_msgs::CameraInfo>(msg);
  *dst_safety_ton = ros::Time::now();
  *dst_status = 1;
}

bool KinectFusion::setTaskNumber( kinect_fusion::SetTask::Request &req ,kinect_fusion::SetTask::Response &res){

  if ((long int)req.task_number < 5 && (long int)req.task_number >= 0){
    ROS_INFO("requested task: %ld", (long int)req.task_number);
    task_number_ = (long int)req.task_number;
    if (task_number_ == 1)
      ROS_INFO("Do task: %ld", (long int)req.task_number);
    else {
      ROS_INFO("Do task: %ld", (long int)req.task_number);
    }
  }
  else{
    ROS_INFO("requested task: %ld is out of range [1-4]",(long int)req.task_number);
  }
  res.send =true;
  return true;
}

void KinectFusion::markerDetect(const cv:: Mat& srs_image, const sensor_msgs::CameraInfoPtr &cam_info_ptr, tf::Transform &dstTF, int marker_id, double marker_size, std::string windows_name ){
  // initialize camera matrix and distortion coefficients
  aruco::CameraParameters camera_param;
  camera_param.CamSize.width = cam_info_ptr ->width;
  camera_param.CamSize.height = cam_info_ptr ->height;

  camera_param.CameraMatrix = cv::Mat::eye(3, 3, CV_32F);
  camera_param.CameraMatrix.at<float>(0,0) = cam_info_ptr ->K[0];
  camera_param.CameraMatrix.at<float>(1,1) = cam_info_ptr ->K[4];
  camera_param.CameraMatrix.at<float>(0,2) = cam_info_ptr ->K[2];
  camera_param.CameraMatrix.at<float>(1,2) = cam_info_ptr ->K[5];

  camera_param.Distorsion = cv::Mat::zeros(1, 5, CV_32F);
  camera_param.Distorsion.at<float>(0,0) = cam_info_ptr ->D[0];
  camera_param.Distorsion.at<float>(0,1) = cam_info_ptr ->D[1];
  camera_param.Distorsion.at<float>(0,2) = cam_info_ptr ->D[2];
  camera_param.Distorsion.at<float>(0,3) = cam_info_ptr ->D[3];
  camera_param.Distorsion.at<float>(0,4) = cam_info_ptr ->D[4];


  cv::Mat img, R;
  srs_image.copyTo(img);
  aruco::MarkerDetector marker_detector;
  std::vector<aruco::Marker> markers;
  tf::Transform Tc_a;

  // detect all markers in an image
  marker_detector.detect(img, markers, camera_param, marker_size, false);
  for (int i = 0; i < markers.size(); i++) {
    markers[i].calculateExtrinsics(marker_size, camera_param);

    cv::Rodrigues(markers[i].Rvec, R);
    // aruco marker wrt camera
    Tc_a.setOrigin(tf::Vector3(markers[i].Tvec.at<float>(0,0), markers[i].Tvec.at<float>(1,0), markers[i].Tvec.at<float>(2,0)));
    Tc_a.setBasis( tf::Matrix3x3(R.at<float>(0,0), R.at<float>(0,1), R.at<float>(0,2),
                                 R.at<float>(1,0), R.at<float>(1,1), R.at<float>(1,2),
                                 R.at<float>(2,0), R.at<float>(2,1), R.at<float>(2,2)));
    ROS_INFO("%s. Marker with id: %d. Tc_a: Position [xyz]: [%lf, %lf, %lf]. Orientation[xyzw]: [%lf, %f, %lf, %lf]",
             windows_name.c_str(),markers[i].id, Tc_a.getOrigin().getX(), Tc_a.getOrigin().getY(), Tc_a.getOrigin().getZ(),
             Tc_a.getRotation().getX(), Tc_a.getRotation().getY(), Tc_a.getRotation().getZ(), Tc_a.getRotation().getW());

    dstTF = Tc_a.inverse();
    ROS_INFO("%s. Marker with id: %d. Ta_c Position [xyz]: [%lf, %lf, %lf]. Orientation[xyzw]: [%lf, %f, %lf, %lf]",
             windows_name.c_str(),markers[i].id, dstTF.getOrigin().getX(), dstTF.getOrigin().getY(), dstTF.getOrigin().getZ(),
             dstTF.getRotation().getX(), dstTF.getRotation().getY(), dstTF.getRotation().getZ(), dstTF.getRotation().getW());

    markers[i].draw(img,cv::Scalar(0,0,255),2);
  }

  cv::imshow(windows_name,img );
  cv::waitKey(1);
}

void KinectFusion::syncPointcloudsCB( const sensor_msgs::PointCloud2ConstPtr &msg1, const sensor_msgs::PointCloud2ConstPtr &msg2,
                                      const sensor_msgs::PointCloud2ConstPtr &msg3, const sensor_msgs::PointCloud2ConstPtr &msg4,
                                      const sensor_msgs::PointCloud2ConstPtr &msg5, const sensor_msgs::PointCloud2ConstPtr &msg6){
  {
    boost::lock_guard<boost::mutex> guard(sync_cb_mutex_);
    cb_clouds_ptr_[0] = boost::const_pointer_cast<sensor_msgs::PointCloud2>(msg1);
    cb_clouds_ptr_[1] = boost::const_pointer_cast<sensor_msgs::PointCloud2>(msg2);
    if (point_topics_.size() > 2)
      cb_clouds_ptr_[2] = boost::const_pointer_cast<sensor_msgs::PointCloud2>(msg3);
    if (point_topics_.size() > 3)
      cb_clouds_ptr_[3] = boost::const_pointer_cast<sensor_msgs::PointCloud2>(msg4);
    if (point_topics_.size() > 4)
      cb_clouds_ptr_[4] = boost::const_pointer_cast<sensor_msgs::PointCloud2>(msg5);
    if (point_topics_.size() > 5){
      cb_clouds_ptr_[5] = boost::const_pointer_cast<sensor_msgs::PointCloud2>(msg6);
    }
    points_status_ = 1;
    safety_tons_points_ = ros::Time::now();
  }
  // All work done in callback. ToDo research for event based callback
  pointcloudsFusion(cb_clouds_ptr_);
}

void KinectFusion::pointcloudsFusion(std::vector<sensor_msgs::PointCloud2::Ptr>  in_clouds_ptr){

  ros::Time tic = ros::Time::now();
  pcl_ros::transformPointCloud ("world", *in_clouds_ptr[0], *transf_clouds_ptr_[0], lr_);
  pcl_ros::transformPointCloud ("world", *in_clouds_ptr[1], *transf_clouds_ptr_[1], lr_);
  pcl::concatenatePointCloud (*transf_clouds_ptr_[0], *transf_clouds_ptr_[1], *fused_cloud_ptr_);

  /// ToDo Test all cases for more than 2 kinects
  if (point_topics_.size() > 2){
    pcl_ros::transformPointCloud ("world", *in_clouds_ptr[2], *transf_clouds_ptr_[2], lr_);
    pcl::concatenatePointCloud (*transf_clouds_ptr_[2], *fused_cloud_ptr_, *fused_cloud_ptr_);
  }
  if (point_topics_.size() > 3){
    pcl_ros::transformPointCloud ("world", *in_clouds_ptr[3], *transf_clouds_ptr_[3], lr_);
    pcl::concatenatePointCloud (*transf_clouds_ptr_[3], *fused_cloud_ptr_, *fused_cloud_ptr_);
  }
  if (point_topics_.size() > 4){
    pcl_ros::transformPointCloud ("world", *in_clouds_ptr[4], *transf_clouds_ptr_[4], lr_);
    pcl::concatenatePointCloud (*transf_clouds_ptr_[4], *fused_cloud_ptr_, *fused_cloud_ptr_);
  }
  if (point_topics_.size() > 5){
    pcl_ros::transformPointCloud ("world", *in_clouds_ptr[5], *transf_clouds_ptr_[5], lr_);
    pcl::concatenatePointCloud (*transf_clouds_ptr_[5], *fused_cloud_ptr_, *fused_cloud_ptr_);
  }
  ROS_INFO("clouds Fusion takes %lf Pointcloud height %d and width %d" , (ros::Time::now() -tic).toSec(), fused_cloud_ptr_->height, fused_cloud_ptr_->width);
  pub_points.publish(*fused_cloud_ptr_);
  pub_points1.publish(*transf_clouds_ptr_[0]);
  pub_points2.publish(*transf_clouds_ptr_[1]);

}
