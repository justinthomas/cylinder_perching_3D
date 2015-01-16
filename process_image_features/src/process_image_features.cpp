// Regular Includes
#include <math.h>
#include <Eigen/Dense>
// #include <Eigen/Geometry>

// ROS Includes
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point.h>
#include <TooN/TooN.h>

// Custom Includes
#include <cylinder_msgs/ImageFeatures.h>
#include <cylinder_msgs/CylinderPose.h>
#include <cylinder_msgs/ParallelPlane.h>
#include "vicon_odom/filter.h"

using namespace std;
using namespace tf;
using namespace Eigen;

#define RED "\e[91m"
#define GREEN "\e[92m"
#define YELLOW "\e[93m"
#define BLUE "\e[94m"
#define MAGENTA "\e[95m"
#define CYAN "\e[96m"
#define RESET "\e[0m"
#define NUM_INF 999999.9

static ros::Publisher pub_features_, pub_pp_, pub_z_in_c;
double r;
static tf::Quaternion imu_q_;
static bool imu_info_(false);
static KalmanFilter kf;
static void pp_features(const double &r, const Eigen::Vector3d &P1_inV, const Eigen::Vector3d &P0, Eigen::Vector3d &s, Eigen::Vector3d &s_sign);
static Eigen::Matrix3d tf2Eigen(tf::Matrix3x3 tfR);
static Eigen::Vector3d tf2Eigen(tf::Vector3 tfvec);
static double filt_alpha;

// Static transformations
static const tf::Matrix3x3 R_CtoB_ = tf::Matrix3x3(-sqrt(2)/2,sqrt(2)/2,0, -sqrt(2)/2,-sqrt(2)/2,0, 0,0,1);
// static const tf::Transform T_CtoB_ = tf::Transform(R_CtoB_, tf::Vector3(-0.05, 0.05, 0));

//IMU buffer
//The delay need to be compensated
std::list<TooN::Vector<3> > acc_buff;
std::list<TooN::Vector<3> > omega_buff;
std::list<TooN::Vector<4> > orientation_buff;
std::list<ros::Time> time_stamp_buff;

// Functions
static void image_features_cb(const cylinder_msgs::ImageFeatures::ConstPtr &msg)
{
  Vector4d im_feats(msg->theta1, msg->theta2, msg->rho1, msg->rho2);
  static Vector4d last_feature_vec = im_feats;

  // ROS_INFO(RED "Features: {%2.2f, %2.2f, %2.2f, %2.2f}" RESET, im_feats[0], im_feats[1], im_feats[2], im_feats[3]);

  // Handle angle wrapping
  while (last_feature_vec[0] < im_feats[0] - M_PI/2)
  {
    last_feature_vec[0] = last_feature_vec[0] + M_PI; // Theta
    last_feature_vec[2] = - last_feature_vec[2];      // Rho
  }
  while (last_feature_vec[0] > im_feats[0] + M_PI/2)
  {
    last_feature_vec[0] = last_feature_vec[0] - M_PI; // Theta
    last_feature_vec[2] = - last_feature_vec[2];      // Rho
  }
  while (last_feature_vec[1] < im_feats[1] - M_PI/2)
  {
    last_feature_vec[1] = last_feature_vec[1] + M_PI; // Theta
    last_feature_vec[3] = - last_feature_vec[3];      // Rho
  }
  while (last_feature_vec[1] > im_feats[1] + M_PI/2)
  {
    last_feature_vec[1] = last_feature_vec[1] - M_PI; // Theta
    last_feature_vec[3] = - last_feature_vec[3];      // Rho
  }

  // Could this make the sign of the rhos change?

  // Simple linear filter on the image features
  im_feats = filt_alpha * im_feats + (1-filt_alpha) * last_feature_vec;
  last_feature_vec = im_feats;

  // ROS_INFO(GREEN "Filt Fet: {%2.2f, %2.2f, %2.2f, %2.2f}" RESET, im_feats[0], im_feats[1], im_feats[2], im_feats[3]);

  double ctheta1, ctheta2, stheta1, stheta2, rho1, rho2;
  ctheta1 = std::cos(im_feats[0]);
  stheta1 = std::sin(im_feats[0]);
  ctheta2 = std::cos(im_feats[1]);
  stheta2 = std::sin(im_feats[1]);
  rho1 = im_feats[2];
  rho2 = im_feats[3];

  tf::Vector3 n1(ctheta1, stheta1, -1*rho1);
  tf::Vector3 n2(ctheta2, stheta2, -1*rho2);

  // We need to make sure that the normal vectors are pointing outwards.
  // The largest rho magnitude should be positive
  if (fabs(n1[2]) >= fabs(n2[2]))
  {
    // The sign of rho1 should be positive (so that the vector points away from the other line in the image)
    if (-n1[2] < 0)
      n1 = -1.0 * n1;

    // The dot product should be negative
    if (tf::tfDot(n1, n2) > 0)
      n2 = -1.0 * n2;
  }
  else
  {
    if (-n2[2] < 0)
      n2 = -1.0 * n2;

    if (tf::tfDot(n1, n2) > 0)
      n1 = -1.0 * n1;
  }

  // ROS_INFO_THROTTLE(1, "n1: {%2.2f, %2.2f, %2.2f}, n2: {%2.2f, %2.2f, %2.2f}", n1[0], n1[1], n1[2], n2[0], n2[1], n2[2]);

  // Delta
  tf::Vector3 Delta(0.5 * (n1 / sqrt(tf::tfDot(n1, n1)) + n2 / sqrt(tf::tfDot(n2, n2))));

  // We know that the cylinder is in front of the camera
  if (Delta[2] < 0)
  {
    Delta = -1 * Delta;
    ROS_WARN("Sign of Delta switched");
  }
  // ROS_INFO_THROTTLE(1, "Delta: {%2.4f, %2.4f, %2.4f}", Delta[0], Delta[1], Delta[2]);

  // s and P0
  tf::Vector3 s = Delta / tfDot(Delta, Delta);
  tf::Vector3 P0 = r * s;
  // ROS_INFO_THROTTLE(10, "Using r = %2.2f", r);

  // ROS_INFO_THROTTLE(1, "\e[33mP0 Estimate: {%2.2f, %2.2f, %2.2f}\e[0m", P0[0], P0[1], P0[2]);

  // tf::Vector3 temp = P0 / sqrt(tf::tfDot(P0, P0));
  // ROS_INFO_THROTTLE(1, "\e[34mP0 Direction {%2.2f, %2.2f, %2.2f}\e[0m", temp[0], temp[1], temp[2]);

  // a, axis of cylinder in camera frame
  tf::Vector3 n2Crossn1 = tf::tfCross(n2, n1);

  // The axis of the cylinder in the camera frame
  tf::Vector3 a = n2Crossn1 / std::sqrt(tf::tfDot(n2Crossn1, n2Crossn1)); // tfVector3Norm(n2Crossn1);
  // ROS_INFO_THROTTLE(1, "\e[0;36ma_est = {%2.2f, %2.2f, %2.2f}\e[0m", a[0], a[1], a[2]);

  // Make sure that the y component (in the camera frame) of a is always positive
  // #### Not a good test
  if (a[1] < 0)
  {
    ROS_INFO(MAGENTA "Axis switched" RESET);
    a = -1.0 * a;
  }

  // Check for the direction of a switching
  /*
  static tf::Vector3 a_last = a;
  if (tf::tfDot(a, a_last) < 0)
  {
    ROS_INFO(MAGENTA "Axis switched" RESET);
    a = -1.0 * a;
  }
  a_last = a;
  */

  // Load the bearing measurements
  tf::Vector3 b1(msg->b1.x, msg->b1.y, msg->b1.z);
  tf::Vector3 b2(msg->b2.x, msg->b2.y, msg->b2.z);

  double delta1 = tfDot(a, b1) * tfDot(P0, b1) / (1 - pow(tfDot(a, b1), 2));
  double delta2 = tfDot(a, b2) * tfDot(P0, b2) / (1 - pow(tfDot(a, b2), 2));
  double delta = (delta1 + delta2) / 2;
  //ROS_INFO("{delta1, delta2} = {%2.2f, %2.2f}", delta1, delta2);
  // ROS_INFO_THROTTLE(1, "delta from processor: %2.2f", delta);
  // ROS_INFO_THROTTLE(1, "\e[0;33mEst: a = {%2.2f, %2.2f, %2.2f}, delta: %2.2f, P0 = {%2.2f, %2.2f, %2.2f}\e[0m",
  //     a[0], a[1], a[2], delta, P0[0], P0[1], P0[2]);

  tf::Vector3 P1 = P0 + (delta * a);
  // ROS_INFO_THROTTLE(1, "P1_est = {%2.2f, %2.2f, %2.2f}", P1[0], P1[1], P1[2]);

  // Publish the info
  cylinder_msgs::CylinderPose cyl_msg;
  cyl_msg.stamp = msg->stamp;
  // cyl_msg.features.filt.theta1 = im_feats[0];
  // cyl_msg.features.filt.theta2 = im_feats[1];
  // cyl_msg.features.filt.rho1 = im_feats[2];
  // cyl_msg.features.filt.rho2 = im_feats[3];
  cyl_msg.a.x = a[0]; cyl_msg.a.y = a[1]; cyl_msg.a.z = a[2];
  cyl_msg.P0.x = P0[0]; cyl_msg.P0.y = P0[1]; cyl_msg.P0.z = P0[2];
  cyl_msg.P1.x = P1[0]; cyl_msg.P1.y = P1[1]; cyl_msg.P1.z = P1[2];
  cyl_msg.n1.x = n1[0]; cyl_msg.n1.y = n1[1]; cyl_msg.n1.z = n1[2];
  cyl_msg.n2.x = n2[0]; cyl_msg.n2.y = n2[1]; cyl_msg.n2.z = n2[2];
  pub_features_.publish(cyl_msg);
}

static void imu_cb(const sensor_msgs::Imu::ConstPtr &msg)
{
	TooN::Vector<4> q;
	q[0] = msg->orientation.w;
	q[1] = msg->orientation.x;
	q[2] = msg->orientation.y;
	q[3] = msg->orientation.z;

  TooN::Vector<3> angular_velocity;
	angular_velocity[0] = msg->angular_velocity.x;
	angular_velocity[1] = msg->angular_velocity.y;
	angular_velocity[2] = msg->angular_velocity.z;

	TooN::Vector<3> acc;
	acc[0] = msg->linear_acceleration.x;
	acc[1] = msg->linear_acceleration.y;
	acc[2] = msg->linear_acceleration.z;

	//buffer IMU to compensate visual delay
  acc_buff.push_front(acc);
	omega_buff.push_front(angular_velocity);
	orientation_buff.push_front(q);
	time_stamp_buff.push_front(msg->header.stamp);

  //tf::quaternionMsgToTF(msg->orientation, imu_q_);
  imu_info_ = true;
}

static void cylinder_pose_cb(const cylinder_msgs::CylinderPose::ConstPtr &msg)
{
  // vision_info_ = true;

  // #### Remove this before actual implementation
  // imu_q_.setEuler(0.0, M_PI/5, 0.0);
  // imu_info_ = true;

  if (!imu_info_)
  {
    ROS_WARN("No IMU info");
    return;
  }

  // Determine the Rotation from world to Camera
  static tf::Vector3 g_in_C, a_in_C, z_in_C, y_in_C, x_in_C;

  //Find the measurement
  //Data should be alligned
  // Find aligned measurement, Time
  double mdt = NUM_INF;
  std::list<TooN::Vector<3> >::iterator    k1 = acc_buff.begin();
  std::list<TooN::Vector<3> >::iterator    k2 = omega_buff.begin();
  std::list<TooN::Vector<4> >::iterator    k3 = orientation_buff.begin();
  std::list<ros::Time>::iterator           k4 = time_stamp_buff.begin();

  //Save alligned IMU measurements
  TooN::Vector<3>   ca;
  TooN::Vector<3>   co;
  TooN::Vector<4>   cq;
  ros::Time kt;
  for (; k1 != acc_buff.end(); k1++, k2++, k3++, k4++)
  {
    double dt = fabs((*k4 - msg->stamp).toSec());
    if (dt < mdt)
    {
      mdt = dt;
      ca  = *k1;
      co  = *k2;
      cq  = *k3;
      kt  = *k4;
    }
    else
    {
      break;
    }
  }
  imu_q_ = tf::Quaternion(cq[1], cq[2], cq[3], cq[0]);

  // Delete redundant measurements
  acc_buff.erase(k1, acc_buff.end());
  omega_buff.erase(k2, omega_buff.end());
  orientation_buff.erase(k3, orientation_buff.end());
  time_stamp_buff.erase(k4, time_stamp_buff.end());

  // Ignore yaw from IMU
  double yaw, pitch, roll;
  tf::Matrix3x3 R_IMU(imu_q_);

  z_in_C = R_CtoB_.transpose() * R_IMU.transpose() * tf::Vector3(0, 0, 1);
  
  geometry_msgs::Point temp;
  temp.x = z_in_C[0];
  temp.y = z_in_C[1];
  temp.z = z_in_C[2];
  pub_z_in_c.publish(temp);
  
  // ROS_INFO_THROTTLE(1, "\e[96mz_in_C: {%2.2f, %2.2f, %2.2f}\e[0m", z_in_C[0], z_in_C[1], z_in_C[2]);

  tf::vector3MsgToTF(msg->a, a_in_C);
  // ROS_INFO_THROTTLE(1, "\e[96ma_in_C: {%2.2f, %2.2f, %2.2f}\e[0m", a_in_C[0], a_in_C[1], a_in_C[2]);
  tf::Vector3 z_x_a = tf::tfCross(z_in_C, a_in_C);
  y_in_C = z_x_a / sqrt(tfDot(z_x_a, z_x_a));

  x_in_C = tf::tfCross(y_in_C, z_in_C);

  Eigen::Matrix3d R_WtoC;
  R_WtoC <<
      x_in_C[0], y_in_C[0], z_in_C[0],
      x_in_C[1], y_in_C[1], z_in_C[1],
      x_in_C[2], y_in_C[2], z_in_C[2];
  // cout << "R_WtoC" << endl << R_WtoC << endl;

  // Determine the Rotation from the virtual camera to the real camera frame
  tf::Vector3 z_pp_in_C = tfCross(a_in_C, y_in_C);
  tf::Matrix3x3 R_VtoC(
      a_in_C[0], y_in_C[0], z_pp_in_C[0],
      a_in_C[1], y_in_C[1], z_pp_in_C[1],
      a_in_C[2], y_in_C[2], z_pp_in_C[2]);

  // The rotation from the virtual frame to the world
  Eigen::Matrix3d R_VtoW = R_WtoC.transpose() * tf2Eigen(R_VtoC);

  // We need this to transform points in the camera to a virtual camera that
  // is fixed to the body of the robot
  tf::Vector3 t_Cam_Shift(0.0, -0.06, 0.02);

  tf::Vector3 P0_inC(msg->P0.x, msg->P0.y, msg->P0.z);
  tf::Vector3 P0_inV = R_VtoC.transpose() * (P0_inC + t_Cam_Shift);

  tf::Vector3 P1_inC(msg->P1.x, msg->P1.y, msg->P1.z);
  tf::Vector3 P1_inV = R_VtoC.transpose() * (P1_inC + t_Cam_Shift);

  // The parallel plane message
  cylinder_msgs::ParallelPlane pp_msg;
  pp_msg.P1.x = P1_inV[0];
  pp_msg.P1.y = P1_inV[1];
  pp_msg.P1.z = P1_inV[2];
  // ROS_INFO_THROTTLE(1, "P1_inV: {%2.2f, %2.2f, %2.2f}", P1_inV(0), P1_inV(1), P1_inV(2));

  // Image feature vector in the virtual frame
  Vector3d P0_inV_Eigen, P1_inV_Eigen, s, s_sign;
  P0_inV_Eigen = tf2Eigen(P0_inV);
  P1_inV_Eigen = tf2Eigen(P1_inV);
  pp_features(r, P0_inV_Eigen, P1_inV_Eigen, s, s_sign);

  pp_msg.s_meas[0] = s(0);
  pp_msg.s_meas[1] = s(1);
  pp_msg.s_meas[2] = s(2);

  //////////////////////
  //  Kalman Filter //
  //////////////////

  // Kalman filter for getting velocity from position measurements
  static ros::Time t_last = msg->stamp;

  double dt = (msg->stamp - t_last).toSec();
  t_last = msg->stamp;

  kf.processUpdate(dt);

  // Switch the measurement vector to our convention for filtering to avoid jumps
  const KalmanFilter::Measurement_t meas(s(0)*s_sign(0), s(1)*s_sign(1), s(2)*s_sign(2));
  kf.measurementUpdate(meas, dt);

  const KalmanFilter::State_t state = kf.getState();
  pp_msg.stamp = msg->stamp;
  // Switch the state vector back to their convention
  pp_msg.s[0]  = state(0) * s_sign(0);
  pp_msg.s[1]  = state(1) * s_sign(1);
  pp_msg.s[2]  = state(2) * s_sign(2);
  pp_msg.sdot[0] = state(3) * s_sign(0);
  pp_msg.sdot[1] = state(4) * s_sign(1);
  pp_msg.sdot[2] = state(5) * s_sign(2);;

  pp_msg.s_sign[0] = s_sign(0);
  pp_msg.s_sign[1] = s_sign(1);
  pp_msg.s_sign[2] = s_sign(2);

  // Load the quaternions
  Eigen::Quaterniond qVtoW(R_VtoW);
  pp_msg.qVtoW.x = qVtoW.x();
  pp_msg.qVtoW.y = qVtoW.y();
  pp_msg.qVtoW.z = qVtoW.z();
  pp_msg.qVtoW.w = qVtoW.w();

  Eigen::Quaterniond qCtoW(R_WtoC.transpose());
  pp_msg.qCtoW.x = qCtoW.x();
  pp_msg.qCtoW.y = qCtoW.y();
  pp_msg.qCtoW.z = qCtoW.z();
  pp_msg.qCtoW.w = qCtoW.w();

  Eigen::Quaterniond qBtoW(R_WtoC.transpose() * tf2Eigen(R_CtoB_).transpose());
  pp_msg.qBtoW.x = qBtoW.x();
  pp_msg.qBtoW.y = qBtoW.y();
  pp_msg.qBtoW.z = qBtoW.z();
  pp_msg.qBtoW.w = qBtoW.w();

  // Publish the message
  pub_pp_.publish(pp_msg);
}

void pp_features(const double &r, const Eigen::Vector3d &P0, const Eigen::Vector3d &P1_inV,
    Eigen::Vector3d &s, Eigen::Vector3d &s_sign)
{
    double A = sqrt(P0.dot(P0) - pow((r), 2));

    // Eigen::Vector3d a_pp = R_VtoC.transpose() * Eigen::Vector3d(a_in_C[0], a_in_C[1], a_in_C[2]);
    // Eigen::Vector3d P0_pp = R_VtoC.transpose() * P0;

    double x0 = 0;
    double y0 = P1_inV(1);
    double z0 = P1_inV(2);

    double a = 1;
    double b = 0;
    double c = 0;

    double alpha = y0*c - z0*b;
    double beta =  z0*a - x0*c;
    double gamma = x0*b - y0*a;

    // Determine the rhos

    double rho1, rho2;
    rho1 = -(r*z0/A - gamma)/
    	sqrt(pow(r*x0/A - alpha, 2) + pow(r*y0/A - beta, 2));

    rho2 = -(r*z0/A + gamma)/
    	sqrt(pow(r*x0/A + alpha, 2) + pow(r*y0/A + beta, 2));

    // Determine the angles
    double ctheta1, ctheta2, stheta1, stheta2;

    ctheta1 = (r*x0/A - alpha)/
    	sqrt(pow(r*x0/A - alpha, 2) + pow(r*y0/A - beta, 2));
    ctheta2 = (r*x0/A + alpha)/
    	sqrt(pow(r*x0/A + alpha, 2) + pow(r*y0/A + beta, 2));
    stheta1 = (r*y0/A - beta)/
    	sqrt(pow(r*x0/A - alpha, 2) + pow(r*y0/A - beta, 2));
    stheta2 = (r*y0/A + beta)/
    	sqrt(pow(r*x0/A + alpha, 2) + pow(r*y0/A + beta, 2));

    double theta1, theta2;
    theta1 = atan2(stheta1, ctheta1);
    theta2 = atan2(stheta2, ctheta2);

    // Handle the case when theta1 or theta2 are less than zero (we want them to both be = M_PI / 2)
    s_sign << 1, 1, 1;

    if (theta1 <= 0)
      s_sign(0) = -1;

    if (theta2 <= 0)
    	s_sign(1) = -1;

    if (fabs(theta1 * s_sign(0) - M_PI / 2) > 0.01 || fabs(theta2 * s_sign(1) - M_PI / 2) > 0.01)
      cout << "Angle is wrong: theta1 = " << theta1 * s_sign(0) << ", theta2 = " << theta2 * s_sign(1) << endl;

    // By default, we are putting this in the x-left, y-up frame
    double u = P1_inV(0) / P1_inV(2);

    // Return the feature vector
    s << rho1, rho2, u;
    // ROS_INFO_THROTTLE(1, "\e[0;33mEstimates: {rho1: %2.2f, rho2: %2.2f, u: %2.2f}\e[0m", rho1, rho2, u);
    // ROS_INFO_THROTTLE(1, "\e[0;33mParallel Plane: {theta1: %2.2f, theta2: %2.2f}\e[0m", theta1, theta2);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "process_image_features");
  ros::NodeHandle n;

  // Parameters
  n.param("cylinder_radius", r, 0.1);
  ROS_INFO("Process_image_features using cylinder_radius = %2.2f", r);

  // For prefiltering the image measurements
  n.param("alpha", filt_alpha, 1.0);
  ROS_INFO("Filter using alpha = %2.2f", filt_alpha);

  // Kalman Filter initalized this way for vicon_odom
  double max_accel;
  n.param("max_accel", max_accel, 4.0);
  ROS_INFO("max_accel = %2.2f", max_accel);

  double dt, camera_fps;
  n.param("camera_fps", camera_fps, 50.0);
  ROS_INFO("Assuming Camera at %2.2f fps", camera_fps);
  ROS_ASSERT(camera_fps > 0.0);
  dt = 1/camera_fps;

  double mnoise1, mnoise2;
  n.param("measurement_noise_rhos", mnoise1, 0.01);
  n.param("measurement_noise_u", mnoise2, 0.1);
  KalmanFilter::State_t proc_noise_diag;
  proc_noise_diag(0) = 0.5*max_accel*dt*dt;
  proc_noise_diag(1) = 0.5*max_accel*dt*dt;
  proc_noise_diag(2) = 0.5*max_accel*dt*dt;
  proc_noise_diag(3) = max_accel*dt;
  proc_noise_diag(4) = max_accel*dt;
  proc_noise_diag(5) = max_accel*dt;
  proc_noise_diag = proc_noise_diag.array().square();
  KalmanFilter::Measurement_t meas_noise_diag;
  meas_noise_diag(0) = mnoise1;
  meas_noise_diag(1) = mnoise1;
  meas_noise_diag(2) = mnoise2;
  meas_noise_diag = meas_noise_diag.array().square();
  kf.initialize(KalmanFilter::State_t::Zero(),
                KalmanFilter::ProcessCov_t::Identity(),
                proc_noise_diag.asDiagonal(),
                meas_noise_diag.asDiagonal());

  // Publishers
  pub_features_ = n.advertise<cylinder_msgs::CylinderPose>("cylinder_pose", 1);
  pub_pp_ = n.advertise<cylinder_msgs::ParallelPlane>("image_features_pp", 1);
  pub_z_in_c = n.advertise<geometry_msgs::Point>("z_in_c", 1);

  // Subscribers
  ros::Subscriber image_features_sub = n.subscribe("/cylinder_detection/cylinder_features", 1, &image_features_cb, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub_imu = n.subscribe("quad_decode_msg/imu", 1, &imu_cb, ros::TransportHints().tcpNoDelay());
  ros::Subscriber cylinder_pose = n.subscribe("cylinder_pose", 1, &cylinder_pose_cb, ros::TransportHints().tcpNoDelay());

  ros::spin();
  return 0;
}

static Eigen::Matrix3d tf2Eigen(tf::Matrix3x3 tfR)
{
  Eigen::Matrix3d R;
  R << tfR[0][0], tfR[0][1], tfR[0][2], tfR[1][0], tfR[1][1], tfR[1][2], tfR[2][0], tfR[2][1], tfR[2][2];
  return R;
}
static Eigen::Vector3d tf2Eigen(tf::Vector3 tfvec)
{
  Eigen::Vector3d vec;
  vec << tfvec[0], tfvec[1], tfvec[2];
  return vec;
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
