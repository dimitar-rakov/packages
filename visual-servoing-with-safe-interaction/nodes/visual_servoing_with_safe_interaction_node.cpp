#include <ros/ros.h>
#include "visual_servoing_with_safe_interaction/visual_servoing_with_safe_interaction.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "visual_servoing_with_safe_interaction_node");
    ros::NodeHandle nh = ros::NodeHandle("~");
    VisualServoingWithSafeInteraction vs;

    if (!vs.init(nh)){
        nh.shutdown();
        return -1;
    }

    ros::Time last_time = ros::Time::now(), init_time = ros::Time::now();
    ros::Duration msr_period(0.0),des_period(0.03), work_period(0.03);
    while(ros::ok())
    {
        init_time = ros::Time::now();
        ros::spinOnce();
        vs.update(ros::Time::now(), work_period);
        msr_period = ros::Time::now() - last_time;
        if (msr_period< des_period)
            (des_period - msr_period).sleep();
        else
            ROS_WARN ("Desired period exceed. Desired period is %lf, last period is %lf", des_period.toSec() ,msr_period.toSec());
        vs.publish();
        last_time = ros::Time::now();
        work_period = last_time - init_time ;
    }
    return 0;
}
