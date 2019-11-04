#ifndef EXAMPLE_PACKAGE_H
#define EXAMPLE_PACKAGE_H

//ROS headers
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <std_srvs/Empty.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

// turtlesim specific msg
#include "turtlesim/Pose.h"

// example_package  specific msg and srv
#include "example_package/SetTask.h"
#include "example_package/SetVelocities.h"

// Open CV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>



class ExamplePackage
{
public:
    ExamplePackage();
    // initilize the node handle
    bool init(ros::NodeHandle &nh);

    //periodicaly called inside of EXAMPLE_PACKAGE. All publisher are there
    void update(const ros::Time& time, const ros::Duration& period);



private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_turtle_msr_pose_;
    ros::Subscriber sub_turtle_cmd_pose_;
    ros::Publisher  pub_cmd_vel_;
    ros::ServiceServer srv_task_number_;
    // subscriber to the raw camera images
    image_transport::Subscriber sub_image_raw_;
    image_transport::ImageTransport image_tran_;
    geometry_msgs::Twist cmd_msg_;

    double msr_pose_x_, msr_pose_y_, msr_pose_theta_;   // measured x, y, theta
    double des_pose_x_, des_pose_y_, des_pose_theta_;   // desired x, y, theta
    double msr_linear_vel_, msr_angular_vel_;           // measured linear and angular velocities
    double des_linear_vel_, des_angular_vel_;           // desired linear and angular velocities
    bool first_msr_received_;                           // flag for first received data from Pose callback
    bool flag_image_ok_;                                // an image is received
    int taskNumber_;                                    // define which task is going to be started

    // mat containers for the all images
    cv::Mat image_raw_, hsv_image_, filtered_image_, green_mask;
    cv::Mat blob_image_, contour_image_, center_of_mass_image_;




    // callback for the turtle pose
    void getTurtlePose(const turtlesim::Pose &msg);

    // set the desired velocities and publish the cmd_msg_
    void commandTurtle(const example_package::SetVelocities &msg);

    // Callback function to image raw topic
    void getImage(const sensor_msgs::ImageConstPtr& msg);

    // Service to set a task number through the terminal
    bool setTaskNumber( example_package::SetTask::Request &req ,example_package::SetTask::Response &res);

    // detect blob
    void extractBlob(cv::Mat InImage);

    // determine the largest blob
    void findBiggestBlob();

};

#endif // EXAMPLE_PACKAGE_H
