#include "ros/ros.h"
#include "project1/Calibrate.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "calibration");
  if (argc != 3)
  {
    ROS_INFO("usage: provide two values for r and N (respectively)");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<project1::Calibrate>("calibration");
  project1::Calibrate srv;
  srv.request.new_r = std::stod(argv[1]);
  srv.request.new_N = std::stoi(argv[2]);

  if (client.call(srv))
  {
    ROS_INFO("Old values: r = %f , N = %d", srv.response.old_r, srv.response.old_N);
  }
  else
  {
    ROS_ERROR("Failed to call service calibration");
    return 1;
  }

  return 0;
}