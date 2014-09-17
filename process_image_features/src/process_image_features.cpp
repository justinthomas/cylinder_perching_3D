#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <math.h>
#include <cylinder_msgs/ImageFeatures.h>
#include <cylinder_msgs/ProcessedImageFeatures.h>

using namespace std;

static ros::Publisher pub_;
double cylinder_radius;

static void image_features_cb(const cylinder_msgs::ImageFeatures::ConstPtr &msg)
{
  cylinder_msgs::ProcessedImageFeatures pif;

  double ctheta1, ctheta2, stheta1, stheta2, rho1, rho2;
  ctheta1 = std::cos(msg->theta1);
  stheta1 = std::sin(msg->theta1);
  ctheta2 = std::cos(msg->theta2);
  stheta2 = std::sin(msg->theta2);
  rho1 = msg->rho1;
  rho2 = msg->rho2;

  tf::Vector3 n1(ctheta1, stheta1, -1*rho1);
  tf::Vector3 n2(ctheta2, stheta2, -1*rho2);

  // ROS_INFO_THROTTLE(1, "n1: {%2.2f, %2.2f, %2.2f}, n2: {%2.2f, %2.2f, %2.2f}", n1[0], n1[1], n1[2], n2[0], n2[1], n2[2]);

  // Delta
  tf::Vector3 Delta(0.5 * (n1 / sqrt(tf::tfDot(n1, n1)) + n2 / sqrt(tf::tfDot(n2, n2))));
  // We know that the cylinder is below us
  if (Delta[2] < 0)
  {
    Delta = -1 * Delta;
    // n1 = -1 * n1;
  }

  // s and P0
  tf::Vector3 s = Delta / tfDot(Delta, Delta);
  tf::Vector3 P0 = cylinder_radius * s;

  // ROS_INFO_THROTTLE(1, "P0 Estimate: {%2.2f, %2.2f, %2.2f}", P0[0], P0[1], P0[2]);
  
  tf::Vector3 temp = P0 / sqrt(tf::tfDot(P0, P0));
  // ROS_INFO_THROTTLE(1, "P0 Direction {%2.2f, %2.2f, %2.2f}", temp[0], temp[1], temp[2]);

  // a, axis of cylinder in camera frame
  tf::Vector3 n2Crossn1 = tf::tfCross(n2, n1);

  // The axis of the cylinder in the camera frame
  tf::Vector3 a = n2Crossn1 / std::sqrt(tf::tfDot(n2Crossn1, n2Crossn1)); // tfVector3Norm(n2Crossn1);
  // ROS_INFO_THROTTLE(1, "\e[0;36ma_est = {%2.2f, %2.2f, %2.2f}\e[0m", a[0], a[1], a[2]);

  tf::Vector3 b(msg->b.x, msg->b.y, msg->b.z);
  double delta = tfDot(a, b) * tfDot(P0, b) / (1 - pow(tfDot(a, b), 2));
  // ROS_INFO_THROTTLE(1, "delta from processor: %2.2f", delta);
  // ROS_INFO_THROTTLE(1, "\e[0;33mEst: a = {%2.2f, %2.2f, %2.2f}, delta: %2.2f, P0 = {%2.2f, %2.2f, %2.2f}\e[0m",
  //     a[0], a[1], a[2], delta, P0[0], P0[1], P0[2]);

  tf::Vector3 P1 = P0 + (delta * a);
  // ROS_INFO_THROTTLE(1, "P1_est = {%2.2f, %2.2f, %2.2f}", P1[0], P1[1], P1[2]);

  // Publish the info
  pif.stamp = msg->stamp;
  pif.a.x = a[0]; pif.a.y = a[1]; pif.a.z = a[2];
  pif.P0.x = P0[0]; pif.P0.y = P0[1]; pif.P0.z = P0[2];
  pif.P1.x = P1[0]; pif.P1.y = P1[1]; pif.P1.z = P1[2];
  pub_.publish(pif);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "process_image_features");
  ros::NodeHandle n;

  // Parameters
  n.param("cylinder_radius", cylinder_radius, 0.1);

  // Publishers
  pub_ = n.advertise<cylinder_msgs::ProcessedImageFeatures>("processed_image_features", 1);

  // Subscribers
  ros::Subscriber image_features_sub = n.subscribe("image_features", 1, &image_features_cb, ros::TransportHints().tcpNoDelay());

  ros::spin();
  return 0;
}

//void print_tfVector3(tf::Vector3 vec)
//{
//	ROS_INFO_THROTTLE(1/5, "Vec: {%2.2f, %2.2f, %2.2f}", vec[0], vec[1], vec[2]);
//	// cout << "Vec: {" << vec[0] << ", " << vec[1] << ", " << vec[2] << "}" << endl;
//}
//
//double tfVector3Norm(tf::Vector3 vec)
//{
//  return std::sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
//}
