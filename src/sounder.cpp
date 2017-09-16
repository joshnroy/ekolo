#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

void soundcallback(const std_msgs::String::ConstPtr& msg)
{
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sound_maker");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/ekolo/cloud", 1, soundcallback);

  ros::spin();

  return 0;
}
