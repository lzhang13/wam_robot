#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

float temp;

void callback(const std_msgs::Float32::ConstPtr& msg)
{
    temp = msg->data;
    ROS_INFO("I heard: [%.3f]", temp);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/biotac_force", 1000, callback);

    ros::spin();

    return 0;
}