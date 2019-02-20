
#include <ros/ros.h>
#include "obstacle_detector/obstacle_detector.h"



int main(int argc, char **argv)
{

    ros::init(argc, argv, "obstacle_detector_node");
    ros::NodeHandle nh = ros::NodeHandle("~");
    obstacle_detector::ObstacleDetector od;

    if (!od.init(nh)){
        nh.shutdown();
        return -1;
    }

    ros::Time last_time = ros::Time::now();
    ros::Time init_time = ros::Time::now();
    ros::Time upd_time = ros::Time::now();
    double des_perion_sec = 0.1;
    if (!nh.getParam("des_period_sec", des_perion_sec))
      ROS_WARN("Parameter des_period_sec was not found. Default value is used: %lf", des_perion_sec);
    ros::Duration des_period(des_perion_sec), msr_period(des_perion_sec), work_period(des_perion_sec);;


    while(ros::ok())
    {
        init_time = ros::Time::now();
        ros::spinOnce();
        upd_time = ros::Time::now();
        od.update(ros::Time::now(), work_period);
        ROS_WARN ("Update time is %lf", (ros::Time::now() - upd_time).toSec());
        msr_period = ros::Time::now() - last_time;
        if (msr_period< des_period)
            (des_period - msr_period).sleep();
        else
            ROS_WARN ("Desired period exceed. Desired period is %lf, last period is %lf", des_period.toSec() ,msr_period.toSec());
        od.publish();
        last_time = ros::Time::now();
        work_period = last_time - init_time ;
    }
    return 0;
}
