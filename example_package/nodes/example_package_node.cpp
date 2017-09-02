#include <ros/ros.h>
#include "example_package/example_package.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "example_package_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);

    ExamplePackage ep;
    ep.init(nh);

    ros::Time init_time = ros::Time::now();
    ros::Time last_time = ros::Time::now();

    while(ros::ok())
    {
        ros::spinOnce();
        ep.update(init_time, ros::Time::now() - last_time);
        last_time = ros::Time::now();
        loop_rate.sleep();
    }
    return 0;
}

