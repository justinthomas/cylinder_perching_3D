#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <math.h>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <cylinder_msgs/ImageFeatures.h>
using namespace std;

static ros::Publisher pub_vision_status_;
static ros::Publisher pub_image_features_;

// Vision Stuff
static tf::Transform T_Cam_to_Body_ = tf::Transform(tf::Matrix3x3(1,0,0, 0,-1,0, 0,0,-1), tf::Vector3(0,0,0));
double cylinder_radius;

// Quadrotor Pose
static geometry_msgs::Point pos_;
static geometry_msgs::Quaternion ori_;
static bool have_odom_ = false;

// Function Declarations
void print_tfVector3(tf::Vector3 vec);

static void odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
  have_odom_ = true;

  pos_ = msg->pose.pose.position;
  ori_ = msg->pose.pose.orientation;

  static tf::Quaternion q;
  static double roll, pitch, yaw;
  tf::quaternionMsgToTF(ori_, q);
  tf::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);

  tf::Vector3 r_cyl_world(0, 0, -2*cylinder_radius);
  tf::Vector3 r_Cam_W(pos_.x, pos_.y, pos_.z);

  double rho1, rho2, ctheta1, ctheta2, stheta1, stheta2;

  tf::Vector3 Axis_of_Cylinder_in_World(1,0,0);

  tf::Matrix3x3 R_B_W(q);
  tf::Vector3 r_B_W(pos_.x, pos_.y, pos_.z);

  tf::Matrix3x3 R_CameraToBody (T_Cam_to_Body_.getBasis());
  tf::Matrix3x3 R_WorldToCamera(R_CameraToBody.transpose() * R_B_W.transpose());

  tf::Vector3 Axis_of_Cylinder_in_Camera(R_WorldToCamera * Axis_of_Cylinder_in_World);
  // This is correct
  // print_tfVector3(Axis_of_Cylinder_in_Camera);

  tf::Transform World_to_Camera_Transform(R_WorldToCamera, R_WorldToCamera * (-1.0 * r_Cam_W));

  // Note: P1 is expressed in the camera frame, not the robot frame
  tf::Vector3 P1(World_to_Camera_Transform * r_cyl_world);
  // print_tfVector3(P1);

  tf::Vector3 Numerator(tf::tfCross(P1, Axis_of_Cylinder_in_Camera));

  // ROS_INFO("Numerator: {%2.2f, %2.2f, %2.2f}", Numerator[0], Numerator[1], Numerator[2]);

  tf::Vector3 Normed_Vec(Numerator / sqrt(tf::tfDot(Numerator, Numerator)));

  //ROS_INFO("Normal Numerator: {%2.2f, %2.2f, %2.2f}", Normal_Numerator[0], Normal_Numerator[1], Normal_Numerator[2]);

  tf::Vector3 h(tf::tfCross(Axis_of_Cylinder_in_Camera, Normed_Vec));

  // ROS_INFO("h:{%2.2f, %2.2f, %2.2f}", h[0], h[1], h[2]);

  double P0_Length = tf::tfDot(P1, h);

  tf::Vector3 P0 = P0_Length*h;


  // ROS_INFO("P0 = {%2.2f, %2.2f, %2.2f}, P1 = {%2.2f, %2.2f, %2.2f}", x0, y0, z0, P1[0], P1[1], P1[2]);

  double A = sqrt(tf::tfDot(P0,P0) - pow((cylinder_radius), 2));

  double x0 = P0[0];
  double y0 = P0[1];
  double z0 = P0[2];

  double a = Axis_of_Cylinder_in_Camera[0];
  double b = Axis_of_Cylinder_in_Camera[1];
  double c = Axis_of_Cylinder_in_Camera[2];

  double alpha = y0*c - z0*b;
  double beta = z0*a - x0*c;
  double gamma = x0*b - y0*a;

  rho1 = (cylinder_radius*z0/A - gamma)/
  	sqrt(pow(cylinder_radius*x0/A - alpha, 2) + pow(cylinder_radius*y0/A - beta, 2));

  rho2 = (cylinder_radius*z0/A + gamma)/
  	sqrt(pow(cylinder_radius*x0/A + alpha, 2) + pow(cylinder_radius*y0/A + beta, 2));

  ctheta1 = (cylinder_radius*x0/A - alpha)/
  	sqrt(pow(cylinder_radius*x0/A - alpha, 2) + pow(cylinder_radius*y0/A - beta, 2));

  ctheta2 = (cylinder_radius*x0/A + alpha)/
  	sqrt(pow(cylinder_radius*x0/A + alpha, 2) + pow(cylinder_radius*y0/A + beta, 2));

  stheta1 = (cylinder_radius*y0/A - beta)/
  	sqrt(pow(cylinder_radius*x0/A - alpha, 2) + pow(cylinder_radius*y0/A - beta, 2));

  stheta2 = (cylinder_radius*y0/A + beta)/
  	sqrt(pow(cylinder_radius*x0/A + alpha, 2) + pow(cylinder_radius*y0/A + beta, 2));

  double theta1 = atan2(stheta1, ctheta1);
  double theta2 = atan2(stheta2, ctheta2);

  // Handle angle wrapping
  if (rho1 < 0)
  {
  	rho1 = - rho1;
  	theta1 = theta1 - M_PI;
  }

  if (rho2 < 0)
  {
  	rho2 = - rho2;
  	theta2 = theta2 - M_PI;
  }

  if (theta1 < -M_PI)
  	theta1 += 2*M_PI;

  if (theta2 < -M_PI)
  	theta2 += 2*M_PI;

  ROS_INFO_THROTTLE(1, "Image coordinates: {rho1: %2.2f, rho2: %2.2f, theta1: %2.0f deg, theta2: %2.0f deg}",
      rho1, rho2, theta1 * 180 / M_PI, theta2 * 180 / M_PI);

  cylinder_msgs::ImageFeatures f;
  f.rho1 = rho1;
  f.theta1 = theta1;
  f.rho2 = rho2;
  f.theta2 = theta2;
  static geometry_msgs::Point P1_msg;
  tf::pointTFToMsg(P1, P1_msg);
  f.P1 = P1_msg;
  pub_image_features_.publish(f);



  //

  //ctheta1, stheta1, ctheta2, stheta2, rho1, rho2, cylinder_radius, b



  	tf::Vector3 n1(ctheta1, stheta1, -1*rho1);
  	tf::Vector3 n2(ctheta2, stheta2, -1*rho2);

  	tf::Vector3 n1_normal(n1/sqrt(tf::tfDot(n1, n1)));

  	tf::Vector3 n2_normal(n2/sqrt(tf::tfDot(n2, n2)));

  	tf::Vector3 Delta(0.5 * (n1_normal + n2_normal));

  	tf::Vector3 Delta_normal(Delta/sqrt(tf::tfDot(Delta, Delta)));

  	tf::Vector3 s_P1_new(Delta/Delta_normal * Delta_normal);

  	//ROS_INFO("s_P1_new:{%2.2f, %2.2f, %2.2f}", s_P1_new[0], s_P1_new[1], s_P1_new[2]);

  	tf::Vector3 P0_new(cylinder_radius*s_P1_new);

  	tf::Vector3 a_P1_new(tf::tfCross(n2, n1)/sqrt(pow(tf::tfCross(n1, n2)[0], 2) + pow(tf::tfCross(n1, n2)[1], 2) + pow(tf::tfCross(n1, n2)[2], 2)));

  	//tf::Vector3 P1_new(P0_new + ((tf::tfDot(a_P1_new + P0_new, b))/(1-pow(tf::tfDot(a_P1_new, b), 2))*a_P1_new));

  	//ROS_INFO("P0:{%2.2f, %2.2f, %2.2f}, P0_new:{%2.2f, %2.2f, %2.2f}", P0[0], P0[1], P0[2], P0_new[0], P0_new[1], P0_new[2]);

  //
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "camera_simulator");
  ros::NodeHandle n;

  // Parameters
  n.param("camera_simulator/radius", cylinder_radius, 0.1);

  // Publishers
  pub_vision_status_ = n.advertise<std_msgs::Bool>("vision_status", 1);
  pub_image_features_ = n.advertise<cylinder_msgs::ImageFeatures>("image_features", 1);

  // Subscribers
  ros::Subscriber sub_odom = n.subscribe("odom", 1, &odom_cb, ros::TransportHints().tcpNoDelay());

  ros::spin();
  return 0;
}

void print_tfVector3(tf::Vector3 vec)
{
	ROS_INFO_THROTTLE(1/5, "Vec: {%2.2f, %2.2f, %2.2f}", vec[0], vec[1], vec[2]);
	// cout << "Vec: {" << vec[0] << ", " << vec[1] << ", " << vec[2] << "}" << endl;
}
