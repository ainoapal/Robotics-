#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "math.h"
#include "std_msgs/String.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "project1/Reset.h"
#include "project1/Calibrate.h"
#include <dynamic_reconfigure/server.h>
#include <project1/parametersConfig.h>

class Odometry
{
public:

  Odometry()
  {
    r = 0.074; //given non-calibrated value : 0.07
    l = 0.200;
    w = 0.169;
    T = 5;
    N = 40;  //given non-calibrated value : 42
    prev_time = ros::Time::now();
    prev_ticks.resize(4,0);
    first_iteration = 0;

    //Parameters from launch file
    n.getParam("/pos_x0",pos_x);
    n.getParam("/pos_y0",pos_y);
    n.getParam("/theta0",theta);

    //publishers
    pub_speed = n.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1000);
    pub_odom = n.advertise<nav_msgs::Odometry>("odom", 1000);

    //dynamic reconfigure to switch type of integration
    f = boost::bind(&Odometry::integration_callback, this, &int_method, _1, _2);
    dynServer.setCallback(f);

    //subscribers
    wheel_speed_sub = n.subscribe("wheel_states", 1000, &Odometry::wheel_statesCallback,this);

    //services
    service = n.advertiseService<project1::Reset::Request, project1::Reset::Response>("reset", 
                boost::bind(&Odometry::reset_callback, this, &pos_x, &pos_y, &theta, _1, _2));
    calibration = n.advertiseService<project1::Calibrate::Request, project1::Calibrate::Response>("calibration", 
                boost::bind(&Odometry::calibration_callback, this, &r, &N, _1, _2));

    ROS_INFO("Odometry listening from topic /wheel_states");

  }

  bool calibration_callback(float* r, int* N, project1::Calibrate::Request  &req, project1::Calibrate::Response &res) {
    res.old_r = *r;
    res.old_N = *N;
    *r = req.new_r;
    *N = req.new_N;
    ROS_INFO("Recalibrating parameters: set r = %f , N = %d \nOld values: r = %f , N = %d", 
        req.new_r, req.new_N, res.old_r, res.old_N);
    return true;
  }  

  bool reset_callback(float* pos_x, float* pos_y, float* theta, project1::Reset::Request  &req, project1::Reset::Response &res) {
    res.old_posx = *pos_x;
    res.old_posy = *pos_y;
    res.old_theta = *theta;
    *pos_x = req.new_posx;
    *pos_y = req.new_posy;
    *theta = req.new_theta;
    first_iteration = 0;
    ROS_INFO("Resetting odometry to pose (%f,%f,%f) - Old pose was: (%f,%f,%f)", 
        req.new_posx, req.new_posy, req.new_theta, res.old_posx, res.old_posy, res.old_theta);
    return true;
  }

  void integration_callback(int* int_method, project1::parametersConfig &config, uint32_t level) {
    *int_method = config.int_method;
    if(*int_method) ROS_INFO("Switch odometry integration to Runge-Kutta");
    else ROS_INFO("Switch odometry integration to Euler");
  }

  void wheel_statesCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    last_time = ros::Time::now();
    delta_t = (last_time - prev_time).toSec();
    if(!first_iteration) {
      for(int i=0; i<prev_ticks.size(); i++) prev_ticks[i] = msg->position[i];
      first_iteration++;
    }

    compute_wheels_speed(msg);

    publish_velocity();

    compute_odometry(int_method);

    publish_odometry(msg);

    //update variables for next iteration
    prev_time = last_time;
    for(int i=0; i<prev_ticks.size(); i++) prev_ticks[i] = msg->position[i];
  }

  void compute_wheels_speed(const sensor_msgs::JointState::ConstPtr& msg) {
    /********************* WHEEL VELOCITIES FROM RADIANS PER MINUTE ***************************
    // wheel vel = (velocity in rad/min : 60 seconds/minute) : gear ratio
    //  [rad/s] =    [rad/min]     * [min/s] * []
    float u1_fl  = (msg->velocity[0]  /60)    /T;
    float u2_fr  = (msg->velocity[1]  /60)    /T;
    float u3_rr  = (msg->velocity[3]  /60)    /T;
    float u4_rl  = (msg->velocity[2]  /60)    /T;
    /******************************************************************************************/

    /**************************** WHEEL VELOCITIES FROM TICKS *********************************/
    // wheel vel = ((d ticks / dt : ticks per revolution) : gear ratio) * radians per revolution
    //  [rad/s] =                 [tick/s]                  * [rev/tick]*  []  * [rad/rev]
    u1_fl  = (msg->position[0] - prev_ticks[0])/delta_t /    N      /   T  * 2*M_PI;
    u2_fr  = (msg->position[1] - prev_ticks[1])/delta_t /    N      /   T  * 2*M_PI;
    u3_rr  = (msg->position[3] - prev_ticks[3])/delta_t /    N      /   T  * 2*M_PI;
    u4_rl  = (msg->position[2] - prev_ticks[2])/delta_t /    N      /   T  * 2*M_PI;

    /********************* COMPUTING ROBOT VELOCITY FROM WHEEL SPEED ************************** 
     *  (w )       ( -1/(l+w)   1/(l+w)   1/(l+w)   -1(l+w) ) 
     *  (vx) = r/4 (    1          1         1         1    ) (u1 u2 u3 u4)T 
     *  (vy)       (   -1          1        -1         1    )
     ******************************************************************************************/
    omega = r/4 * ( (-1/(l+w))*u1_fl + ( 1/(l+w))*u2_fr +
                    ( 1/(l+w))*u3_rr + (-1/(l+w))*u4_rl ) ;
    ROS_INFO("Angular velocity: [%f]", omega);

    vx = r/4 * (  1*u1_fl + 1*u2_fr +  1*u3_rr + 1*u4_rl ) ;
    vy = r/4 * ( -1*u1_fl + 1*u2_fr + -1*u3_rr + 1*u4_rl ) ;
    ROS_INFO("Linear speed: (%f,%f)", vx, vy);
  }

  void publish_velocity() {
    //Publish results
    geometry_msgs::TwistStamped speed;
    speed.twist.linear.x  = vx;
    speed.twist.linear.y  = vy;
    speed.twist.angular.z = omega;
    pub_speed.publish(speed);
  }

  void compute_odometry(int int_method) {
    //Dynamic reconfigure parameter changes integration method
    if(!int_method) {
      //Euler
      pos_x += (vx * cos(theta) - vy * sin(theta)) * delta_t;
      pos_y += (vx * sin(theta) + vy * cos(theta)) * delta_t;
      theta += omega * delta_t;
    }
    else {
      //Runge-Kutta
      pos_x += (vx * cos(theta + omega * delta_t / 2) - vy * sin(theta + omega * delta_t / 2)) * delta_t;
      pos_y += (vx * sin(theta + omega * delta_t / 2) + vy * cos(theta + omega * delta_t / 2)) * delta_t;
      theta += omega * delta_t;
    }
    ROS_INFO("Odometry: (%f,%f,%f)", pos_x, pos_y, theta);
  }

  void publish_odometry(const sensor_msgs::JointState::ConstPtr& msg) {
    //Publish results
    nav_msgs::Odometry odom;
    odom.pose.pose.position.x  = pos_x;
    odom.pose.pose.position.y  = pos_y;
    tf2::Quaternion q;
    q.setRPY(0,0,theta);
    tf2::convert(q, odom.pose.pose.orientation);
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y  = vy;
    odom.twist.twist.angular.z = omega;
    odom.header.stamp = ros::Time::now();
    odom.header.seq = msg->header.seq;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    pub_odom.publish(odom);
  }

private:
  //Declaration of variables
  ros::NodeHandle n;

  float r; //wheel radius
  float l; //wheel position along x
  float w; //wheel position along y
  int T; //gear ratio
  int N; //encoders resolution --> CPR

  float pos_x;
  float pos_y;
  float theta;

  float u1_fl;
  float u2_fr;
  float u3_rr;
  float u4_rl;

  float vx;
  float vy;
  float omega;
     
  ros::Time prev_time;
  ros::Time last_time;
  double delta_t;
  std::vector<float> prev_ticks;
  int first_iteration;

  //publishers
  ros::Publisher pub_speed;
  ros::Publisher pub_odom; 

  //dynamic reconfigure to switch type of integration
  int int_method;
  dynamic_reconfigure::Server<project1::parametersConfig> dynServer;
  dynamic_reconfigure::Server<project1::parametersConfig>::CallbackType f;

  //subscribers
  ros::Subscriber wheel_speed_sub;

  //services
  ros::ServiceServer service;
  ros::ServiceServer calibration;
};


int main(int argc, char **argv) {
  ros::init(argc, argv, "odometry");
  
  Odometry odometry;

  ros::spin();
  return 0;
}
