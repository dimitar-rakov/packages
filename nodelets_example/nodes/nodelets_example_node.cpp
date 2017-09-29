#include <ros/ros.h>
#include "nodelets_example/nodelets_example.h"

int main(int argc, char **argv)
{

    ros::init(argc, argv, "nodelets_example_node");
    ros::NodeHandle nh = ros::NodeHandle("~");
    examples::NodeletsExample ne;

    if (!ne.init(nh)){
        nh.shutdown();
        return -1;
    }

    ros::Time last_time = ros::Time::now(), init_time = ros::Time::now();
    double des_perion_sec;
    if (!nh.getParam("des_period_sec", des_perion_sec))
      ROS_WARN("Parameter des_period_sec was not found. Default value is used: %lf", des_perion_sec);
    ros::Duration des_period(des_perion_sec), msr_period(des_perion_sec), work_period(des_perion_sec);;

    while(ros::ok())
    {
        init_time = ros::Time::now();
        ros::spinOnce();
        ne.update(ros::Time::now(), work_period);
        msr_period = ros::Time::now() - last_time;
        if (msr_period< des_period)
         (des_period - msr_period).sleep();
        else
          ROS_WARN_COND(des_perion_sec >0, "Desired period exceed. Desired period is %lf, last period is %lf", des_period.toSec() ,msr_period.toSec());
        last_time = ros::Time::now();
        work_period = last_time - init_time ;
    }
    return 0;
}
