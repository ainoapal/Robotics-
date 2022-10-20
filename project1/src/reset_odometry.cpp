#include "ros/ros.h"
#include "project1/Reset.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "reset_odometry");
  if (argc != 4)
  {
    ROS_INFO("usage: provide a new pose (x,y,theta) in order to reset the odometry");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<project1::Reset>("reset");
  project1::Reset srv;
  srv.request.new_posx = std::stod(argv[1]);
  srv.request.new_posy = std::stod(argv[2]);
  srv.request.new_theta = std::stod(argv[3]);

  if (client.call(srv))
  {
    ROS_INFO("Old pose: (%f,%f,%f)", srv.response.old_posx, srv.response.old_posy, srv.response.old_theta);
  }
  else
  {
    ROS_ERROR("Failed to call service reset");
    return 1;
  }

  return 0;
}