#include "odometry_frame_publisher.h"
#include "ros/ros.h"

int main(int argc,char** argv){
  ros::init(argc, argv, "teleop");
  pioneer_tools::OdometryFramePublisher odometry_publisher;

  return 0;
}
