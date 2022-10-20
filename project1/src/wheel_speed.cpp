#include "ros/ros.h"
#include "math.h"
#include "std_msgs/String.h"
#include "geometry_msgs/TwistStamped.h"
#include "project1/WheelSpeed.h"

class Wheel_speed
{
  public:

    Wheel_speed()
    {
      r = 0.074; //wheel radius //given non-calibrated value : 0.07
      l = 0.200; //wheel position along x
      w = 0.169; //wheel position along y
      T = 5; //gear ratio
      N = 40; //encoders resolution --> CPR  //given non-calibrated value : 42

      //publishers
      pub_wheelSpeed = n.advertise<project1::WheelSpeed>("wheels_rpm", 1000);

      //subscribers
      wheel_speed_sub = n.subscribe("cmd_vel", 1000, &Wheel_speed::cmd_velCallback, this);
      ROS_INFO("Waiting for topic cmd_vel");
    }

    void cmd_velCallback(const geometry_msgs::TwistStamped::ConstPtr& speed) {
      vx = speed->twist.linear.x;
      vy = speed->twist.linear.y;
      omega = speed->twist.angular.z;

      /*******************   COMPUTING WHEEL SPEED FROM ROBOT VELOCITY ************************** 
       *  (u1)       ( -l-w    1   -1 ) 
       *  (u2) = 1/r (  l+w    1    1 ) (w vx vy)T 
       *  (u3)       (  l+w    1   -1 )
       *  (u4)       ( -l-w    1    1 )
       *   **************************************************************************************/
      u1_fl  =  ((-l-w)*omega + 1*vx + -1*vy)/r;
      u2_fr  =  ( (l+w)*omega + 1*vx +  1*vy)/r;
      u3_rr  =  ( (l+w)*omega + 1*vx + -1*vy)/r;
      u4_rl  =  ((-l-w)*omega + 1*vx +  1*vy)/r;

      /********************* CONVERT WHEEL VELOCITIES ***************************/
      //    vel   = (velocity in rad/s * gear ratio * 60 seconds/minute) 
      //  [rad/min] =    [rad/s]       * [] *   [s/min]
      u1_fl         =     u1_fl        *  T *     60;
      u2_fr         =     u2_fr        *  T *     60;
      u3_rr         =     u3_rr        *  T *     60;
      u4_rl         =     u4_rl        *  T *     60;
      // to get rpm, one should multiply the obtained value *2pi, but the given data is in rad/min
      // so we will publish the data in rad/min in order to have the same measure unit as in the bags 

      //Publish results
      project1::WheelSpeed wheelSpeed;
      wheelSpeed.rpm_fl = u1_fl;
      wheelSpeed.rpm_fr = u2_fr;
      wheelSpeed.rpm_rr = u3_rr;
      wheelSpeed.rpm_rl = u4_rl;
      pub_wheelSpeed.publish(wheelSpeed);
      ROS_INFO("Wheels speed: (%f,%f,%f,%f)", u1_fl, u2_fr, u3_rr, u4_rl);

    }

  private:
    //Declaration of variables
    ros::NodeHandle n;

    float r; //wheel radius
    float l; //wheel position along x
    float w; //wheel position along y
    int T; //gear ratio
    int N; //encoders resolution --> CPR

    float vx;
    float vy;
    float omega;

    float u1_fl;
    float u2_fr;
    float u3_rr;
    float u4_rl;

    //publishers
    ros::Publisher pub_wheelSpeed;

    //subscribers
    ros::Subscriber wheel_speed_sub;
};


int main(int argc, char **argv) {
  ros::init(argc, argv, "wheel_state");
  
  Wheel_speed wheel_speed;

  ros::spin();

  return 0;
}
