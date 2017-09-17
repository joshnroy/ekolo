#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <iostream>
#include <AL/al.h>
#include <AL/alc.h>
#include <AL/alut.h>
#include <unistd.h>
#include <math.h>

using namespace std;

void soundcallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sound_maker");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/ekolo/cloud", 1, soundcallback);

  ros::spin();

  return 0;
}
