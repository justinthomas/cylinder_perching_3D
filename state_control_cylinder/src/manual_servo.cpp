// Regular Includes
// #include <Eigen/Dense>
// #include <math.h>
// #include <stdio.h>

// ROS Related Includes
#include <ros/ros.h>
#include <std_msgs/Float64.h>

// Custom Includes
#include <quadrotor_msgs/PWMCommand.h>
#include <quadrotor_msgs/OutputData.h>

// using namespace std;
// using namespace Eigen;
// using namespace tf;

// Publishers & services
static ros::Publisher pub_pwm_command_;

static void output_data_cb(const quadrotor_msgs::OutputData::ConstPtr &msg)
{
  if(msg->radio_channel[7] > 0)
  {
    quadrotor_msgs::PWMCommand pwm_cmd;
    pwm_cmd.pwm[0] = 0.1;
    pub_pwm_command_.publish(pwm_cmd);
  }
  else
  {
    quadrotor_msgs::PWMCommand pwm_cmd;
    pwm_cmd.pwm[0] = 0.4;
    pub_pwm_command_.publish(pwm_cmd);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "manual_servo");
  ros::NodeHandle n("~");

  // Publishers
  pub_pwm_command_ = n.advertise<quadrotor_msgs::PWMCommand>("pwm_cmd", 1);

  // Subscribers
  ros::Subscriber sub_output = n.subscribe("output_data", 1, &output_data_cb, ros::TransportHints().tcpNoDelay());

  // Spin
  ros::spin();

  return 0;
}
