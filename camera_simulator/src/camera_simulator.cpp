#include <ros/ros.h>
#include <math.h>
#include <Eigen/Dense>
#include <iostream>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_broadcaster.h>
#include <cylinder_msgs/ImageFeatures.h>

using namespace tf;
using namespace Eigen;
using namespace std;

// static ros::Publisher pub_vision_status_;
static ros::Publisher pub_image_features_;

// Vision Stuff
static const tf::Matrix3x3 R_CtoB_ = tf::Matrix3x3(sqrt(2)/2,sqrt(2)/2,0, sqrt(2)/2,-sqrt(2)/2,0, 0,0,-1);
static tf::Transform T_Cam_to_Body_ = tf::Transform(R_CtoB_, tf::Vector3(0,0,0));
double r;

// Quadrotor Pose
static geometry_msgs::Point pos_;
static geometry_msgs::Quaternion ori_;
static bool have_odom_ = false;

// Function Declarations
void print_tfVector3(tf::Vector3 vec);
Eigen::Matrix3d hat(Eigen::Vector3d vec);
Eigen::Vector3d Vec3TfToEigen(tf::Vector3 vec);
void wrapAngle(double &angle);

static void odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
  have_odom_ = true;

  pos_ = msg->pose.pose.position;
  ori_ = msg->pose.pose.orientation;

  static tf::Quaternion q;
  static double roll, pitch, yaw;
  tf::quaternionMsgToTF(ori_, q);
  tf::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);

  static tf::Vector3 r_Cyl_World(0, 0, -2*r);
  tf::Vector3 r_Cam_W(pos_.x, pos_.y, pos_.z);

  static double rho1, rho2;

  // Note: These are coupled through yaw.  Can't change one without changing the other
  tf::Vector3 a_World(1,0,0);
  tf::Matrix3x3 R_B_W(q);
  //
  
  tf::Vector3 r_B_W(pos_.x, pos_.y, pos_.z);

  tf::Matrix3x3 R_CameraToBody (T_Cam_to_Body_.getBasis());
  tf::Matrix3x3 R_WorldToCamera(R_CameraToBody.transpose() * R_B_W.transpose());

  tf::Vector3 a_Cam(R_WorldToCamera * a_World);
  // This is correct
  // ROS_INFO_THROTTLE(1, "\e[0;36ma:  {%2.2f, %2.2f, %2.2f}\033[0m", a_Cam[0], a_Cam[1], a_Cam[2]);

  // Correct
  tf::Transform T_WorldToCamera(R_WorldToCamera, R_WorldToCamera * (-1 * r_Cam_W));
  // print_tfVector3(T_WorldToCamera * tf::Vector3(1, 0, 0);

  // Note: P1 is expressed in the camera frame, not the robot frame
  tf::Vector3 P1(T_WorldToCamera * r_Cyl_World);
  // ROS_INFO_THROTTLE(1, "\e[0;36mP1: {%2.2f, %2.2f, %2.2f}\033[0m", P1[0], P1[1], P1[2]);

  tf::Vector3 P1_x_a(tf::tfCross(P1, a_Cam));
  // ROS_INFO("P1_x_a: {%2.2f, %2.2f, %2.2f}", P1_x_a[0], P1_x_a[1], P1_x_a[2]);

  tf::Vector3 h(tf::tfCross(
        a_Cam,
        P1_x_a / sqrt(tf::tfDot(P1_x_a, P1_x_a))));

  // ROS_INFO("h:{%2.2f, %2.2f, %2.2f}", h[0], h[1], h[2]);
  // ROS_INFO_THROTTLE(1, "h_mag: {%2.2f}", std::sqrt(tf::tfDot(h, h)));

  // P0 length = tf::tfDot(P1, h)
  tf::Vector3 P0 = tf::tfDot(P1, h) * h;

  // This is only temp
  tf::Vector3 temp = P0 / sqrt(tf::tfDot(P0, P0));
  // ROS_INFO_THROTTLE(1, "P0 =         {%2.2f, %2.2f, %2.2f}", temp[0], temp[1], temp[2]); //,, P1 = {%2.2f, %2.2f, %2.2f}", P0[0], P0[1], P0[2], P1[0], P1[1], P1[2]);

  double A = sqrt(tf::tfDot(P0,P0) - r*r);

  double theta1, theta2;

  // Let's keep these variables in this scope (later we will want to use b)
  {
    // Note that the convention is different than the modern day standard.  See Espiau, 1992.
    double x0 = -P0[0];
    double y0 = -P0[1];
    double z0 = P0[2];

    // Also, different convention
    double a = -a_Cam[0];
    double b = -a_Cam[1];
    double c = a_Cam[2];

    double alpha = y0*c - z0*b;
    double beta = z0*a - x0*c;
    double gamma = x0*b - y0*a;

    rho1 = (r*z0/A - gamma)/
    	sqrt(pow(r*x0/A - alpha, 2) + pow(r*y0/A - beta, 2));

    rho2 = (r*z0/A + gamma)/
    	sqrt(pow(r*x0/A + alpha, 2) + pow(r*y0/A + beta, 2));

    double ctheta1, ctheta2, stheta1, stheta2;

    ctheta1 = (r*x0/A - alpha)/
    	sqrt(pow(r*x0/A - alpha, 2) + pow(r*y0/A - beta, 2));

    ctheta2 = (r*x0/A + alpha)/
    	sqrt(pow(r*x0/A + alpha, 2) + pow(r*y0/A + beta, 2));

    stheta1 = (r*y0/A - beta)/
    	sqrt(pow(r*x0/A - alpha, 2) + pow(r*y0/A - beta, 2));

    stheta2 = (r*y0/A + beta)/
    	sqrt(pow(r*x0/A + alpha, 2) + pow(r*y0/A + beta, 2));

    theta1 = atan2(stheta1, ctheta1);
    theta2 = atan2(stheta2, ctheta2);
  }

  // Handle the case when rho1 or rho2 are less than zero
  // if (rho1 < 0)
  // {
  // 	rho1 = - rho1;
  // 	theta1 = theta1 - M_PI;
  // }
  // if (rho2 < 0)
  // {
  // 	rho2 = - rho2;
  // 	theta2 = theta2 - M_PI;
  // }

  // Wrap to -Pi to Pi
  wrapAngle(theta1);
  wrapAngle(theta2);
  
  // ROS_INFO_THROTTLE(1, "\e[0;36mp:  {rho1: %2.2f, rho2: %2.2f, theta1: %2.0f deg, theta2: %2.0f deg}\033[0m",
  //  rho1, rho2, theta1 * 180 / M_PI, theta2 * 180 / M_PI);

  ////////////////////
  // Determine Pt1 //
  //////////////////

  // From eq (9)
  // double gamma = std::sqrt(tf::tfDot(P0, P0) - r*r); // Note: this is the same as A in Chaumette 1994
  Vector3d a = Vec3TfToEigen(a_Cam);
  // eq (15) to determine the direction of Pt0
  Vector3d d = (A * hat(a) - r * Matrix3d::Identity()).inverse() * hat(a) * Vec3TfToEigen(P0);
  Vector3d Pt0 = A * d;  // eq (8)
  if (Pt0(2) < 0)
    ROS_WARN("Cylinder is behind camera!");
  // ROS_INFO_THROTTLE(1, "Z component of Pt0: %2.2f", Pt0(2));
  
  // Vector3d temp = Vec3TfToEigen(P1 - P0) / std::sqrt(tf::tfDot(P1 - P0, P1 - P0));
  // ROS_INFO_THROTTLE(1, "a: {%2.2f, %2.2f, %2.2f}, P1 - P0 direction: {%2.2f, %2.2f, %2.2f}", a(0), a(1), a(2), temp(0), temp(1), temp(2));
  // This verifies that these vectors are parallel 

  double delta = a.dot(Vec3TfToEigen(P1 - P0)); // std::sqrt(tf::tfDot(P1 - P0, P1 - P0));
  // ROS_INFO_THROTTLE(1, "delta from sim = %2.2f", delta);

  Vector3d Pt1 = Pt0 + delta * a;
  Vector3d b = Pt1 / sqrt(Pt1(0)*Pt1(0) + Pt1(1)*Pt1(1) + Pt1(2)*Pt1(2));
  // ROS_INFO_THROTTLE(1, "b = {%2.2f, %2.2f, %2.2f}", b(0), b(1), b(2));

  cylinder_msgs::ImageFeatures f;
  f.stamp = ros::Time::now();
  f.rho1 = rho1;
  f.theta1 = theta1;
  f.rho2 = rho2;
  f.theta2 = theta2;
  f.b.x = b(0); f.b.y = b(1); f.b.z = b(2);
  pub_image_features_.publish(f);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "camera_simulator");
  ros::NodeHandle n;

  // Parameters
  n.param("cylinder_radius", r, 0.1);

  // Publishers
  // pub_vision_status_ = n.advertise<std_msgs::Bool>("vision_status", 1);
  pub_image_features_ = n.advertise<cylinder_msgs::ImageFeatures>("image_features", 1);

  // Subscribers
  ros::Subscriber sub_odom = n.subscribe("odom", 1, &odom_cb, ros::TransportHints().tcpNoDelay());

  ros::spin();
  return 0;
}

Eigen::Matrix3d hat(Eigen::Vector3d vec)
{
  Eigen::Matrix3d M;
  M << 0.0, -vec[2], vec[1], vec[2], 0.0, -vec[0], -vec[1], vec[0], 0.0;
  return M;
}

void print_tfVector3(tf::Vector3 vec)
{
	ROS_INFO_THROTTLE(1/5, "\e[36m" "Vec: {%2.2f, %2.2f, %2.2f}" "\e[0m", vec[0], vec[1], vec[2]);
	// cout << "Vec: {" << vec[0] << ", " << vec[1] << ", " << vec[2] << "}" << endl;
}

Vector3d Vec3TfToEigen(tf::Vector3 vec)
{
  Eigen::Vector3d vec2;
  vec2 << vec[0], vec[1], vec[2];
  return vec2;
}

void wrapAngle(double &angle)
{
  while (angle < -M_PI)
  	angle += 2*M_PI;

  while (angle > M_PI)
  	angle -= 2*M_PI;
}
