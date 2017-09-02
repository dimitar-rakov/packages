#include "example_package/example_package.h"



ExamplePackage::ExamplePackage() : image_tran_(nh_)
{ }

bool ExamplePackage::init(ros::NodeHandle &nh)
{
    // Ros node handle for the class
    nh_ = nh;
    first_msr_received_ =false;
    flag_image_ok_ =false;
    // subsriber for the turtle pose
    sub_turtle_msr_pose_= nh_.subscribe("/turtle1/pose", 100, &ExamplePackage::getTurtlePose, this);
    sub_turtle_cmd_pose_= nh_.subscribe("/turtle1/new_cmd_pose", 100, &ExamplePackage::commandTurtle, this);
    sub_image_raw_ = image_tran_.subscribe("/usb_cam/image_raw", 10, &ExamplePackage::getImage, this);
    srv_task_number_ = nh_.advertiseService("/set_task_number", &ExamplePackage::setTaskNumber, this);

    // publisher to turtle velocity
    pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);


    ROS_INFO ("ExamplePackage is initialized");
    return true;

}

void ExamplePackage::update(const ros::Time& time, const ros::Duration& period){
  ROS_INFO ("ok");

    if (first_msr_received_){
        std::cout << "MsrPose:  "<<"x: "<<msr_pose_x_<<"   y: "<<msr_pose_y_<<"    theta: "<<msr_pose_theta_<<"\n";
        std::cout << "Velocities: "<<"msr linear velocity: "<<msr_linear_vel_<<"   msr angular velocity: "<<msr_angular_vel_<<"\n";
        std::cout << "Velocities: "<<"des linear velocity: "<<cmd_msg_.linear.x<<"   des angular velocity: "<<cmd_msg_.angular.z<<"\n";
    }

    if (flag_image_ok_){
        cv::cvtColor(image_raw_, hsv_image_, CV_BGR2HSV);
        cv::GaussianBlur(hsv_image_, filtered_image_, cv::Size(9, 9), 1, 1);
        // green color for the blobs
        cv::inRange(filtered_image_, cv::Scalar(40, 100, 100), cv::Scalar(80, 255, 255), green_mask);
        extractBlob(green_mask);
        imshow("Center of blob", center_of_mass_image_);
        cv::waitKey(1);
    }


}

void ExamplePackage::getTurtlePose(const turtlesim::Pose &msg){

        msr_pose_x_= msg.x;
        msr_pose_y_= msg.y;
        msr_pose_theta_= msg.theta;
        msr_angular_vel_= msg.angular_velocity;
        msr_linear_vel_= msg.linear_velocity;
        first_msr_received_ =true;
}

void ExamplePackage::commandTurtle(const example_package::SetVelocities &msg){
   if (msg.id==1)
        cmd_msg_.linear.x = msg.linear_vel[0];

   else if (msg.id==2)
           cmd_msg_.angular.z = msg.angular_vel[2];
   else {
       cmd_msg_.linear.x = msg.linear_vel[0];
       cmd_msg_.angular.z = msg.angular_vel[2];
   }

   pub_cmd_vel_.publish(cmd_msg_);
}

void ExamplePackage::getImage(const sensor_msgs::ImageConstPtr& msg){
     // pointer on OpenCV image
        cv_bridge::CvImagePtr cv_ptr;

        try
        {
            // transform ROS image into OpenCV image
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            image_raw_ = cv_ptr->image;
            flag_image_ok_ = true;

        }
        catch (cv_bridge::Exception& e)		// throw an error msg. if conversion fails
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
 }

bool ExamplePackage::setTaskNumber( example_package::SetTask::Request &req ,example_package::SetTask::Response &res){

    if ((long int)req.task_number < 5 && (long int)req.task_number >= 0){
        ROS_INFO("requested task: %ld", (long int)req.task_number);
        taskNumber_ = (long int)req.task_number;
        if (taskNumber_ == 1)
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

void ExamplePackage::extractBlob(cv::Mat InImage){

   cv::dilate(InImage, blob_image_, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);
   // Create a structuring element
   int erosion_size = 6;
   cv::Mat element = getStructuringElement(cv::MORPH_CROSS,
                 cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
                 cv::Point(erosion_size, erosion_size) );

   // Apply erosion or dilation on the image
   cv::erode(blob_image_,blob_image_,element);  // dilate(image,dst,element);

   /// Find biggest blob
   std::vector<std::vector<cv::Point> > contours;
   // Find contours
   blob_image_.copyTo(contour_image_);
   cv::findContours( contour_image_, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE );

   // Get the moments
   std::vector<cv::Moments> mu(contours.size() );
   double biggest_countourArea = 50.0;       // to filter the small features and noise
   cv::Moments biggest_mu;
   cv::Point2f mc;

   for( int i = 0; i < contours.size(); i++ )
   {
       mu[i] = cv::moments( contours[i], false );
       if (biggest_countourArea < contourArea(contours[i]) )
       {
           biggest_countourArea = contourArea(contours[i]);
           biggest_mu = mu[i];
       }
   }

   //  Get the mass centers:
   mc = cv::Point2f( biggest_mu.m10/biggest_mu.m00 , biggest_mu.m01/biggest_mu.m00 );
   if(!std::isnan (mc.x) && !std::isnan (mc.y)){
       std::cout << " The biggest countour is at: "<< mc<<"\n";
   }
   // Mat drawing used to show mask + blob's centers
   cv::cvtColor(blob_image_,center_of_mass_image_,CV_GRAY2RGB);

   // draw a red cross at the center of the blob
   cv::line(center_of_mass_image_, cv::Point2f (mc.x-10, mc.y+10), cv::Point2f (mc.x+10, mc.y-10), cv::Scalar(0,0,255),2);
   cv::line(center_of_mass_image_, cv::Point2f (mc.x+10, mc.y+10), cv::Point2f (mc.x-10, mc.y-10), cv::Scalar(0,0,255),2);

}

