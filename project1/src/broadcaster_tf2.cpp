#include "ros/ros.h"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include "nav_msgs/Odometry.h"

class TfBroad
{
public:
  TfBroad() {
    sub = n.subscribe("/odom", 1000, &TfBroad::callback, this);
  }

  void callback(const nav_msgs::Odometry::ConstPtr& odom){
    // set header
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "odom";
    transformStamped.child_frame_id = "base_link";
    // set x,y
    transformStamped.transform.translation.x = odom->pose.pose.position.x;
    transformStamped.transform.translation.y = odom->pose.pose.position.y;
    transformStamped.transform.translation.z = odom->pose.pose.position.z;
    // set theta
    transformStamped.transform.rotation = odom->pose.pose.orientation;
    // send transform
    br.sendTransform(transformStamped);
  }

private:
  ros::NodeHandle n; 
  tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  ros::Subscriber sub;
};


int main(int argc, char **argv) {
  ros::init(argc, argv, "tf2_broad");
  TfBroad my_tf_broadcaster;
  ros::spin();
  return 0;
}