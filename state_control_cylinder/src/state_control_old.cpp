#include <ros/ros.h>
#include <controllers_manager/Transition.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Quaternion.h>
#include <velocity_tracker/GoalCommand.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <math.h>
#include <iostream>
#include <tf/transform_broadcaster.h>
using namespace std;

// #define SAFETY_ON

enum controller_state
{
  INIT,
  TAKEOFF,
  HOVER,
  LINE_TRACKER,
  LINE_TRACKER_YAW,
  VELOCITY_TRACKER,
  VISION_CONTROL,
  LAND,
};

static int vision_control_button = 34; // Cycle
static bool vision_info_ = false;
static bool imu_info_ = false;
static int estop_button = 26;  // Stop
static int takeoff_button = 27;  // Play
static int motors_on_button = 28;  // Rec
static int line_tracker_button = 29;  // Track L
static int velocity_tracker_button = 30; // Track R
static int hover_button = 31; // Marker Set
static int line_tracker_yaw_button = 24; // Rewind 

static int num_robots = 1; // This must be less than the size of the next line
static int quad_selectors[] = {5,6,7}; // The buttons of solo 

static double xoff, yoff, zoff, yaw_off;
static int quad_num_;

static enum controller_state state_ = INIT;
 
static ros::Publisher pub_goal_min_jerk_;
static ros::Publisher pub_goal_distance_;
static ros::Publisher pub_goal_velocity_;
static ros::Publisher pub_motors_;
static ros::Publisher pub_estop_;
static ros::Publisher pub_goal_yaw_;
static ros::Publisher pub_bearings_level_;
static ros::ServiceClient srv_transition_;
static ros::Publisher pub_vision_status_;

// Vision Stuff
static tf::Transform T_Cam_to_Body_ = tf::Transform(tf::Matrix3x3(1,0,0, 0,-1,0, 0,0,-1), tf::Vector3(0,0,0));

// Quadrotor Pose
static geometry_msgs::Point pos_;
static geometry_msgs::Quaternion ori_;
static geometry_msgs::Quaternion imu_q_;
static bool have_odom_ = false;

// Strings
static const std::string line_tracker_distance("line_tracker/LineTrackerDistance");
static const std::string line_tracker("line_tracker/LineTracker");
static const std::string line_tracker_yaw("line_tracker/LineTrackerYaw");
static const std::string velocity_tracker_str("velocity_tracker/VelocityTrackerYaw");

// Function Declarations
void hover_in_place();
void print_tfVector3(tf::Vector3 vec);

// Callbacks and functions
static void nanokontrol_cb(const sensor_msgs::Joy::ConstPtr &msg)
{
  if(msg->buttons[estop_button])
  {
    // Publish the E-Stop command
    ROS_WARN("E-STOP");
    std_msgs::Empty estop_cmd;
    pub_estop_.publish(estop_cmd);

    // Disable motors
    ROS_WARN("Disarming motors...");
    std_msgs::Bool motors_cmd;
    motors_cmd.data = false;
    pub_motors_.publish(motors_cmd);
  }
  
  if(state_ == INIT) 
  {
    if (!have_odom_)
    {
      ROS_INFO("Waiting for Odometry!");
      return;
    }
  
    // Motors on (Rec) 
    if(msg->buttons[motors_on_button])
    {
      ROS_INFO("Sending enable motors command");
      std_msgs::Bool motors_cmd;
      motors_cmd.data = true;
      pub_motors_.publish(motors_cmd);
    }
    
    // Take off (Play)
    if(msg->buttons[takeoff_button])
    {
      state_ = TAKEOFF;
      ROS_INFO("Initiating launch sequence...");

      geometry_msgs::Point goal;
      goal.x = pos_.x;
      goal.y = pos_.y;
      goal.z = pos_.z + 0.10;
      pub_goal_distance_.publish(goal);
      usleep(100000);
      controllers_manager::Transition transition_cmd;
      transition_cmd.request.controller = line_tracker_distance;
      srv_transition_.call(transition_cmd);
    }
    else
      ROS_INFO("Waiting to take off.  Press Rec to enable motors and Play to Take off.");
  }
  else
  {
    // This is executed every time the midi controller changes
    switch(state_)
    {    
      case VELOCITY_TRACKER:
        {
        velocity_tracker::GoalCommand goal;
        goal.x = msg->axes[0] * fabs(msg->axes[0]) / 2;
        goal.y = msg->axes[1] * fabs(msg->axes[1]) / 2;
        goal.z = msg->axes[2] * fabs(msg->axes[2]) / 2;
        goal.yaw_dot = msg->axes[3] * fabs(msg->axes[3]) / 2;
       
        pub_goal_velocity_.publish(goal);
        ROS_INFO("Velocity Command: (%1.4f, %1.4f, %1.4f, %1.4f)", goal.x, goal.y, goal.z, goal.yaw_dot);
        }
        break;

      default:
        break;    
    }

    // Determine whether or not the quad is selected
    bool selected = (num_robots == 1);
    for (int i = 0; i < num_robots; i++)
    {
      selected = selected || (msg->buttons[quad_selectors[i]] && i == quad_num_);
    }

    // Hover
    if(msg->buttons[hover_button])  // Marker Set
    {
      hover_in_place(); 
    }
    // Line Tracker
    else if(selected && msg->buttons[line_tracker_button] && (state_ == HOVER || state_ == LINE_TRACKER || state_ == TAKEOFF))
    {
      state_ = LINE_TRACKER;
      ROS_INFO("Engaging controller: LINE_TRACKER");
      geometry_msgs::Point goal;
      goal.x = 2*msg->axes[0] + xoff;
      goal.y = 2*msg->axes[1] + yoff;
      goal.z = msg->axes[2] + .9 + zoff;
      pub_goal_min_jerk_.publish(goal);
      controllers_manager::Transition transition_cmd;
      transition_cmd.request.controller = line_tracker;
      srv_transition_.call(transition_cmd);
    }
    // Line Tracker Yaw
    else if(selected && msg->buttons[line_tracker_yaw_button] && (state_ == HOVER || state_ == LINE_TRACKER_YAW || state_ == TAKEOFF))
    {
      state_ = LINE_TRACKER_YAW;
      ROS_INFO("Engaging controller: LINE_TRACKER_YAW");
      std_msgs::Float64 goal;
      goal.data = M_PI * msg->axes[3] + yaw_off;
      pub_goal_yaw_.publish(goal);
      controllers_manager::Transition transition_cmd;
      transition_cmd.request.controller = line_tracker_yaw;
      srv_transition_.call(transition_cmd);
    }
    // Velocity Tracker
    else if(selected && msg->buttons[velocity_tracker_button] && state_ == HOVER)
    {
      // Note: We do not want to send a goal of 0 if we are 
      // already in the velocity tracker controller since it 
      // could cause steps in the velocity.

      state_ = VELOCITY_TRACKER;
      ROS_INFO("Engaging controller: VELOCITY_TRACKER");

      velocity_tracker::GoalCommand goal;
      goal.x = 0;
      goal.y = 0;
      goal.z = 0;
      goal.yaw_dot = 0;
      pub_goal_velocity_.publish(goal);
      controllers_manager::Transition transition_cmd;
      transition_cmd.request.controller = velocity_tracker_str;
      srv_transition_.call(transition_cmd);
    }
    // Vision Control
    else if(selected && msg->buttons[vision_control_button] && state_ == HOVER && vision_info_ && imu_info_)
    {
      state_ = VISION_CONTROL;
      ROS_INFO("Engaging controller: VELOCITY_TRACKER for Vision-based control");
      /* 
      for (unsigned int idx = 0; idx < bc_bearing_goal_.size(); idx++)
      { 
        ROS_INFO("Bearing %d is : (%2.2f, %2.2f, %2.2f) with scale = %2.2f", idx,
            bc_bearing_goal_[idx][0], bc_bearing_goal_[idx][1], bc_bearing_goal_[idx][2], bc_scale_goal_[idx]);
      }
      */

      velocity_tracker::GoalCommand goal;
      goal.x = 0;
      goal.y = 0;
      goal.z = 0;
      goal.yaw_dot = 0;
      pub_goal_velocity_.publish(goal);
      controllers_manager::Transition transition_cmd;
      transition_cmd.request.controller = velocity_tracker_str;
      srv_transition_.call(transition_cmd);
      
      std_msgs::Bool vision_status;
      vision_status.data = true;
      pub_vision_status_.publish(vision_status);
    }
  }
}

// static void bearings_cb(ibvs_formation_bearing::bearing b)
// { 
//   static tf::Quaternion q;
//   static tf::Matrix3x3 R;
//   static tf::Transform T, T_Body_to_level_rp;

//   if(imu_info_)
//   { 
//     bearings_ = b.bearings;
  
//     std::vector<tf::Vector3> bearing(bearings_.size());

//     // Convert the quaternion to RPY
//     static double roll, pitch, yaw;
//     tf::quaternionMsgToTF(imu_q_, q);
//     tf::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);
//     // ROS_INFO_THROTTLE(1, "RPY = (%lf, %lf, %lf)", roll, pitch, yaw);

//     // Create a rotation matrix with yaw = zero
//     // T transforms bearings from the camera to the level plane of the robot 
//     R.setEulerYPR(0, pitch, roll);
//     T_Body_to_level_rp = tf::Transform(R, tf::Vector3(0,0,0));
//     T = T_Body_to_level_rp * T_Cam_to_Body_;

//     // Loop through each bearing
//     bearings_level_.resize(bearings_.size());
//     for (unsigned int idx = 0; idx < bearings_.size(); idx++)
//     {
//       tf::vector3MsgToTF(bearings_[idx].vector, bearing[idx]);
//       bearings_level_[idx] = T * bearing[idx];
//       tf::vector3TFToMsg(bearings_level_[idx], b.bearings[idx].vector);
//     }

//     // Safety catch if the bearing estimate has changed too much since last time
//     #ifdef SAFETY_ON
//       if(state_ == VISION_CONTROL)    
//       {
//         if (bearings_level_set_)
//         {
//           for (unsigned int idx = 0; idx < bearing.size(); idx++)
//           {
//             // The Sum of Squared Differences between the last two bearings
//             static std::vector<double> bearing_ssd(bearings_.size());
//             bearing_ssd[idx] = pow(last_bearings_[idx][0] - bearings_level_[idx][0], 2)
//               + pow(last_bearings_[idx][1] - bearings_level_[idx][1], 2)
//               + pow(last_bearings_[idx][2] - bearings_level_[idx][2], 2);

//             if (bearing_ssd[idx] > 0.15) 
//             {
//               ROS_WARN("Bearing change too large for %d viewing %d. Skipping bearing...\nLast Scaled Bearing: {x: %f, y: %f, z: %f}\nThis Scaled Bearing: {x: %f, y: %f, z: %f}",
//                   quad_num_, idx,
//                   last_bearings_[idx][0], last_bearings_[idx][1], last_bearings_[idx][2],
//                   bearings_level_[idx][0], bearings_level_[idx][1], bearings_level_[idx][2]);
//               return;
//             }
//           }
//         }
//       }
//     #endif

//     // Update bearings_
//     pub_bearings_level_.publish(b);
//     bearings_level_set_ = true;
//     last_bearings_ = bearings_level_;
//     last_bearings_time_ = ros::Time::now();
//   }
// }

static void odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
  have_odom_ = true;

  pos_ = msg->pose.pose.position;
  ori_ = msg->pose.pose.orientation;

  static tf::Quaternion q;
  static double roll, pitch, yaw;
  tf::quaternionMsgToTF(ori_, q);
  tf::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);

  // TF broadcaster to broadcast the quadrotor frame
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(pos_.x, pos_.y, pos_.z) );
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/simulator", "/quadrotor"));

  // Start here

  double cylinder_radius = 0.1;

  tf::Vector3 r_Cylinder_World(0, 0, -2*cylinder_radius);

  tf::Vector3 r_Cam_W (pos_.x, pos_.y, pos_.z);

  double rho1, rho2, ctheta1, ctheta2, stheta1, stheta2;

  tf::Vector3 Axis_of_Cylinder_in_World(1,0,0);

  tf::Matrix3x3 R_B_W (q);
  tf::Vector3 r_B_W(pos_.x, pos_.y, pos_.z);

  tf::Matrix3x3 R_Camera_Body (T_Cam_to_Body_.getBasis());


  tf::Matrix3x3 R_W_Cam(R_Camera_Body.transpose() * R_B_W.transpose());

  tf::Vector3 Axis_of_Cylinder_in_Camera(R_W_Cam * Axis_of_Cylinder_in_World);
  // This is correct
  // print_tfVector3(Axis_of_Cylinder_in_Camera);

  tf::Transform World_to_Camera_Transform(R_W_Cam, R_W_Cam * (-1.0 * r_Cam_W));

  // Note: P1 is expressed in the camera frame, not the robot frame
  tf::Vector3 P1(World_to_Camera_Transform * r_Cylinder_World);
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

  ROS_INFO("Image coordinates: {rho1: %2.2f, rho2: %2.2f, theta1: %2.0f deg, theta2: %2.0f deg}", rho1, rho2, theta1 * 180 / M_PI, theta2 * 180 / M_PI);

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

  static tf::Matrix3x3 R; 
  R.setEulerYPR(0, pitch, roll);
  R.getRotation(q);
  q.normalize();

#ifdef SAFETY_ON
  // Position and attitude Safety Catch
  if (state_ == VISION_CONTROL && (abs(pos_.x) > 2 || abs(pos_.y) > 1.5 || pos_.z > 2.5 || pos_.z < 0.5))
  {
    ROS_WARN("Position safety catch initiated from VISION_CONTROL...");
    hover_in_place();
  }
  else if (state_ == VISION_CONTROL && abs(q.getAngle()) > M_PI/10)
  {
    ROS_WARN("Attitude safety catch initiated from VISION_CONTROL...");
    hover_in_place();
  } 
  else if (state_ == VELOCITY_TRACKER && (abs(pos_.x) > 2.2 || abs(pos_.y) > 1.8|| pos_.z > 3.5 || pos_.z < 0.2 || q.getAngle() > M_PI/10))
  {
    ROS_WARN("Safety Catch initiated from VELOCITY_TRACKER...");
    hover_in_place();
  }
  
  // // Check timing of last vision message and switch to hover if it has been too long
  // static double vision_timeout = 0.3;
  // if (state_ == VISION_CONTROL && (ros::Time::now().toSec() - last_bearings_time_.toSec()) > vision_timeout)
  // {
  //   ROS_WARN("Vision message timeout. Time since last message: %2.2f seconds", ros::Time::now().toSec() - last_bearings_time_.toSec());
  //   hover_in_place();
  // }

#endif
}
  
static void imu_cb(const sensor_msgs::Imu::ConstPtr &msg)
{
  imu_q_ = msg->orientation;
  imu_info_ = true;
}

void hover_in_place()
{
  state_ = HOVER;
  ROS_INFO("Hovering in place...");

  geometry_msgs::Point goal;
  goal.x = pos_.x;
  goal.y = pos_.y;
  goal.z = pos_.z;
  pub_goal_distance_.publish(goal);
  usleep(100000);
  controllers_manager::Transition transition_cmd;
  transition_cmd.request.controller = line_tracker_distance; 
  srv_transition_.call(transition_cmd);
  
  std_msgs::Bool vision_status;
  vision_status.data = false;
  pub_vision_status_.publish(vision_status);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "state_control");
  ros::NodeHandle n;

  // Now, we need to set the formation offsets for this robot
  n.param("state_control/offsets/x", xoff, 0.0);
  n.param("state_control/offsets/y", yoff, 0.0);
  n.param("state_control/offsets/z", zoff, 0.0);
  n.param("state_control/offsets/yaw", yaw_off, 0.0); 
  n.param("state_control/quad_num", quad_num_, 0);
  // n.param("state_control/vision_gains/bearing", bearing_gain_, 0.4);
  // n.param("state_control/vision_gains/range", range_gain_, 0.4);
  // n.param("state_control/vision_gains/yaw", yaw_gain_, 0.4);
  ROS_INFO("Quad #%d using offsets: {xoff: %2.2f, yoff: %2.2f, zoff: %2.2f, yaw_off: %2.2f}", quad_num_, xoff, yoff, zoff, yaw_off);
  //ROS_INFO("Vision gains are: {bearing: %2.2f, range: %2.2f, yaw: %2.2f}", bearing_gain_, range_gain_, yaw_gain_);

  // Publishers
  srv_transition_= n.serviceClient<controllers_manager::Transition>("controllers_manager/transition");
  pub_goal_min_jerk_ = n.advertise<geometry_msgs::Vector3>("controllers_manager/line_tracker/goal", 1);
  pub_goal_distance_ = n.advertise<geometry_msgs::Vector3>("controllers_manager/line_tracker_distance/goal", 1);
  pub_goal_velocity_ = n.advertise<velocity_tracker::GoalCommand>("controllers_manager/velocity_tracker/vel_cmd", 1);
  pub_goal_yaw_ = n.advertise<std_msgs::Float64>("controllers_manager/line_tracker_yaw/goal", 1);
  pub_motors_ = n.advertise<std_msgs::Bool>("motors", 1);
  pub_estop_ = n.advertise<std_msgs::Empty>("estop", 1);
  pub_vision_status_ = n.advertise<std_msgs::Bool>("vision_status", 1);

  // Subscribers
  ros::Subscriber sub_odom = n.subscribe("odom", 1, &odom_cb, ros::TransportHints().tcpNoDelay());
  // ros::Subscriber sub_bearings = n.subscribe("getCircle/bearings", 1, &bearings_cb, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub_imu = n.subscribe("quad_decode_msg/imu", 1, &imu_cb, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub_nanokontrol = n.subscribe("/nanokontrol2", 1, nanokontrol_cb, ros::TransportHints().tcpNoDelay());
  // ros::Subscriber sub_consensus = n.subscribe("/range_consensus", 1, &consensus_cb, ros::TransportHints().tcpNoDelay());

  // Disabling the motors to be safe
  ROS_INFO("Disabling motors for launch");
  std_msgs::Bool motors_cmd;
  motors_cmd.data = false;
  pub_motors_.publish(motors_cmd);

  ros::spin();

  return 0;
}

void print_tfVector3(tf::Vector3 vec)
{
	ROS_INFO_THROTTLE(1/5, "Vec: {%2.2f, %2.2f, %2.2f}", vec[0], vec[1], vec[2]);
	// cout << "Vec: {" << vec[0] << ", " << vec[1] << ", " << vec[2] << "}" << endl;
}