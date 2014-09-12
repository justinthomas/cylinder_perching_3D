#include <ros/ros.h>
#include <controllers_manager/Transition.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Quaternion.h>
#include <quadrotor_msgs/FlatOutputs.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <math.h>
#include <iostream>
#include "nano_kontrol2.h"
// #include <trajectory.h>
#include <tf/transform_broadcaster.h>
#include <cylinder_msgs/ImageFeatures.h>
using namespace std;

#define SAFETY_ON

enum controller_state
{
  INIT,
  TAKEOFF,
  HOVER,
  LINE_TRACKER_JERK,
  LINE_TRACKER_YAW,
  LINE_TRACKER_DISTANCE,
  VELOCITY_TRACKER,
  VISION_CONTROL,
  PERCH,
  RECOVER,
  PREP_TRAJ,
  TRAJ,
  NONE,
};

// Variables and parameters
double xoff, yoff, zoff, yaw_off, mass_;
bool safety, cut_motors_after_traj;
static geometry_msgs::Point home_;

// Stuff for trajectory
/* 
#include <string>
traj_type traj;
ros::Time traj_start_time;
double traj_time;
static ros::Publisher pub_goal_trajectory_;
quadrotor_msgs::PositionCommand traj_goal_;
static const std::string trajectory_tracker_str("trajectory_tracker/TrajectoryTracker");
void updateTrajGoal();
static std::string traj_filename;
*/

// States
static enum controller_state state_ = INIT;

// Publishers & services
static ros::Publisher pub_goal_min_jerk_;
static ros::Publisher pub_goal_distance_;
static ros::Publisher pub_goal_velocity_;
static ros::Publisher pub_motors_;
static ros::Publisher pub_estop_;
static ros::Publisher pub_goal_yaw_;
static ros::Publisher pub_info_bool_;
static ros::ServiceClient srv_transition_;
static ros::Publisher pub_vision_status_;

// Vision Stuff
static tf::Transform T_Cam_to_Body_ = tf::Transform(tf::Matrix3x3(1,0,0, 0,-1,0, 0,0,-1), tf::Vector3(0,0,0));


// Quadrotor Pose
static geometry_msgs::Point pos_;
static geometry_msgs::Vector3 vel_;
static geometry_msgs::Quaternion ori_;
static geometry_msgs::Quaternion imu_q_;
static bool have_odom_(false), imu_info_ (false), vision_info_(false);

// Strings
static const std::string line_tracker_distance("line_tracker/LineTrackerDistance");
static const std::string line_tracker_jerk("line_tracker/LineTrackerJerk");
static const std::string line_tracker_yaw("line_tracker/LineTrackerYaw");
static const std::string velocity_tracker_str("velocity_tracker/VelocityTrackerYaw");

// Function Declarations
void hover_in_place();
void hover_at(const geometry_msgs::Point goal);
void go_home();
void go_to(const quadrotor_msgs::FlatOutputs goal);
void print_tfVector3(tf::Vector3 vec);

// Callbacks and functions
static void nanokontrol_cb(const sensor_msgs::Joy::ConstPtr &msg)
{
  cut_motors_after_traj = msg->buttons[7];

  for(int i=0; i<35; i++)
  {
    if(msg->buttons[i]==1)
    {
      cout << "Button " << i << " was pressed" << endl;
    }
  }

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
    if(msg->buttons[play_button])
    {
      state_ = TAKEOFF;
      ROS_INFO("Initiating launch sequence...");

      // home_ has global scope
      home_.x = pos_.x;
      home_.y = pos_.y;
      home_.z = pos_.z + 0.10;
      pub_goal_distance_.publish(home_);
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
          quadrotor_msgs::FlatOutputs goal;
          goal.x = msg->axes[0] * fabs(msg->axes[0]) / 2;
          goal.y = msg->axes[1] * fabs(msg->axes[1]) / 2;
          goal.z = msg->axes[2] * fabs(msg->axes[2]) / 2;
          goal.yaw = msg->axes[3] * fabs(msg->axes[3]) / 2;

          pub_goal_velocity_.publish(goal);
          ROS_INFO("Velocity Command: (%1.4f, %1.4f, %1.4f, %1.4f)", goal.x, goal.y, goal.z, goal.yaw);
        }
        break;

      default:
        break;
    }

    // Determine whether or not the quad is selected
    bool selected = true;

    // Hover
    if(msg->buttons[hover_button])  // Marker Set
    {
      hover_in_place();
    }
    // Line Tracker
    else if(selected && msg->buttons[line_tracker_button] && (state_ == HOVER || state_ == LINE_TRACKER_JERK || state_ == TAKEOFF))
    {
      state_ = LINE_TRACKER_JERK;
      ROS_INFO("Engaging controller: LINE_TRACKER_JERK");
      geometry_msgs::Point goal;
      goal.x = 2*msg->axes[0] + xoff;
      goal.y = 2*msg->axes[1] + yoff;
      goal.z = 2*msg->axes[2] + 2.0 + zoff;
      pub_goal_min_jerk_.publish(goal);
      controllers_manager::Transition transition_cmd;
      transition_cmd.request.controller = line_tracker_jerk;
      srv_transition_.call(transition_cmd);
    }
    // Line Tracker Yaw
    else if(selected && msg->buttons[line_tracker_yaw_button] && (state_ == HOVER || state_ == LINE_TRACKER_YAW || state_ == TAKEOFF))
    {
      quadrotor_msgs::FlatOutputs goal;
      goal.x = 2*msg->axes[0] + xoff;
      goal.y = 2*msg->axes[1] + yoff;
      goal.z = 2*msg->axes[2] + 2.0 + zoff;
      goal.yaw = M_PI * msg->axes[3] + yaw_off;
      go_to(goal);
    }
    // Velocity Tracker
    else if(selected && msg->buttons[velocity_tracker_button] && state_ == HOVER)
    {
      // Note: We do not want to send a goal of 0 if we are
      // already in the velocity tracker controller since it
      // could cause steps in the velocity.

      state_ = VELOCITY_TRACKER;
      ROS_INFO("Engaging controller: VELOCITY_TRACKER");

      quadrotor_msgs::FlatOutputs goal;
      goal.x = 0;
      goal.y = 0;
      goal.z = 0;
      goal.yaw = 0;
      pub_goal_velocity_.publish(goal);
      controllers_manager::Transition transition_cmd;
      transition_cmd.request.controller = velocity_tracker_str;
      srv_transition_.call(transition_cmd);
    }
    /*
    else if(msg->buttons[traj_button] && state_ == HOVER)
    {
      // traj[t_idx][flat_out][deriv]
      //
      // Load the trajectory
      int flag = loadTraj(traj_filename.c_str(), traj);

      // If there are any errors
      if (flag != 0)
      {
        hover_in_place();
        ROS_WARN("Couldn't load %s.  Error: %d.  Hovering in place...", traj_filename.c_str(), flag);
      }
      else
      {
        state_ = PREP_TRAJ;
        ROS_INFO("Loading Trajectory.  state_ == PREP_TRAJ;");

        // Updates traj goal to allow for correct initalization of the trajectory
        traj_start_time = ros::Time::now();
        updateTrajGoal();

        quadrotor_msgs::FlatOutputs goal;
        goal.x = traj[0][0][0] + xoff;
        goal.y = traj[0][1][0] + yoff;
        goal.z = traj[0][2][0] + zoff;
        goal.yaw = traj[0][3][0] + yaw_off;

        pub_goal_yaw_.publish(goal);
        controllers_manager::Transition transition_cmd;
        transition_cmd.request.controller = line_tracker_yaw;
        srv_transition_.call(transition_cmd);
      }
    }
    else if(msg->buttons[play_button] && state_ == PREP_TRAJ)
    {
      // If we are ready to start the trajectory
      if ( sqrt( pow(traj_goal_.position.x + xoff - pos_.x, 2) + pow(traj_goal_.position.y + yoff - pos_.y, 2) + pow(traj_goal_.position.z + zoff - pos_.z, 2) ) < .03 ||
           sqrt( pow(vel_.x,2) + pow(vel_.y,2) + pow(vel_.z,2) ) < 0.05)
      {
        ROS_INFO("Starting Trajectory");

        state_ = TRAJ;

        // Publish the trajectory signal
        std_msgs::Bool traj_on_signal;
        traj_on_signal.data = true;
        pub_info_bool_.publish(traj_on_signal);

        traj_start_time = ros::Time::now();

        updateTrajGoal();

        pub_goal_trajectory_.publish(traj_goal_);
        controllers_manager::Transition transition_cmd;
        transition_cmd.request.controller = trajectory_tracker_str;
        srv_transition_.call(transition_cmd);
      }
      else
      {
        ROS_WARN("Not ready to start trajectory.");
      }
    }
    */
    // Vision Control
    else if(selected && msg->buttons[vision_control_button] && state_ == HOVER && vision_info_ && imu_info_)
    {
      state_ = VISION_CONTROL;
    }
  }
}

/*
void updateTrajGoal()
{
  ros::Time current_time = ros::Time::now();
  ros::Duration delta_time = current_time - traj_start_time;
  traj_time = delta_time.toSec();

  unsigned long i = traj_time * 1000;

  if (i > traj.size()-1)
  {
    ROS_INFO("Trajectory completed.");


    // At this point, we could call switch to a perch state, which
    // could trigger a recover if the perch was not successful
    geometry_msgs::Point goal;
    goal.x = traj[traj.size()-1][0][0] + xoff;
    goal.y = traj[traj.size()-1][1][0] + yoff;
    goal.z = traj[traj.size()-1][2][0] + zoff;
    state_ = PERCH;
    // hover_at(goal);

    std_msgs::Bool traj_on_signal;
    traj_on_signal.data = false;
    pub_info_bool_.publish(traj_on_signal);
  }
  else
  {
    traj_goal_.position.x = traj[i][0][0] + xoff;
    traj_goal_.position.y = traj[i][1][0] + yoff;
    traj_goal_.position.z = traj[i][2][0] + zoff;

    traj_goal_.velocity.x = traj[i][0][1];
    traj_goal_.velocity.y = traj[i][1][1];
    traj_goal_.velocity.z = traj[i][2][1];

    traj_goal_.acceleration.x = traj[i][0][2];
    traj_goal_.acceleration.y = traj[i][1][2];
    traj_goal_.acceleration.z = traj[i][2][2];

    traj_goal_.jerk.x = traj[i][0][3];
    traj_goal_.jerk.y = traj[i][1][3];
    traj_goal_.jerk.z = traj[i][2][3];

    traj_goal_.yaw = traj[i][3][0] + yaw_off;
    traj_goal_.yaw_dot = traj[i][3][1];

    traj_goal_.kx[0] = traj[i][4][0];
    traj_goal_.kx[1] = traj[i][4][1];
    traj_goal_.kx[2] = traj[i][4][2];
    traj_goal_.kv[0] = traj[i][4][3];
    traj_goal_.kv[1] = traj[i][4][4];
    traj_goal_.kv[2] = traj[i][4][5];

    ROS_INFO_THROTTLE(1, "Gains: kx: {%2.1f, %2.1f, %2.1f}, kv: {%2.1f, %2.1f, %2.1f}",
        traj[i][4][0],
        traj[i][4][1],
        traj[i][4][2],
        traj[i][4][3],
        traj[i][4][4],
        traj[i][4][5]);
  }
}
*/

static void imu_cb(const sensor_msgs::Imu::ConstPtr &msg)
{
  imu_q_ = msg->orientation;
  imu_info_ = true;
}

static void image_update_cb(const cylinder_msgs::ImageFeatures::ConstPtr &msg)
{
  vision_info_ = true;
}
void hover_at(const geometry_msgs::Point goal)
{
  state_ = HOVER;
  ROS_INFO("Hovering at (%2.2f, %2.2f, %2.2f)", goal.x, goal.y, goal.z);

  pub_goal_distance_.publish(goal);
  usleep(100000);
  controllers_manager::Transition transition_cmd;
  transition_cmd.request.controller = line_tracker_distance;
  srv_transition_.call(transition_cmd);
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
}

void go_home()
{
  state_ = LINE_TRACKER_DISTANCE;
  pub_goal_distance_.publish(home_);
  controllers_manager::Transition transition_cmd;
  transition_cmd.request.controller = line_tracker_distance;
  srv_transition_.call(transition_cmd);
}

void go_to(const quadrotor_msgs::FlatOutputs goal)
{
  state_ = LINE_TRACKER_YAW;
  ROS_INFO("Engaging controller: LINE_TRACKER_YAW");
  pub_goal_yaw_.publish(goal);
  controllers_manager::Transition transition_cmd;
  transition_cmd.request.controller = line_tracker_yaw;
  srv_transition_.call(transition_cmd);
}

static void odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
  have_odom_ = true;

  pos_ = msg->pose.pose.position;
  vel_ = msg->twist.twist.linear;
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


  // If we are currently executing a trajectory, update the setpoint
  /*
  if (state_ == TRAJ)
  {
    updateTrajGoal();
    pub_goal_trajectory_.publish(traj_goal_);
  }

  if (state_ == PERCH)
  {
    if (cut_motors_after_traj)
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

      pub_goal_trajectory_.publish(traj_goal_);

      //// We want the desired position and velocity errors to be zero
      //traj_goal_.position = pos_;
      //traj_goal_.velocity = vel_;

      //traj_goal_.acceleration = tf::Vector3(0,0,- 9.810);
      //// Specify the orientation
      //float des_acc_mag = 0.0001;
      //float acc_mag = std::sqrt(
      //    traj_goal_.position.x*traj_goal_.position.x
      //    + traj_goal_.position.y*traj_goal_.position.y
      //    + traj_goal_.position.z*traj_goal_.position.z);

      //traj_goal_.acceleration.x = des_acc_mag * traj_goal_.acceleration.x / acc_mag;
      //traj_goal_.acceleration.y = des_acc_mag * traj_goal_.acceleration.y / acc_mag;
      //traj_goal_.acceleration.z = des_acc_mag * traj_goal_.acceleration.z / acc_mag - mass_ * 9.81; // Note: we are subtracting gravity

      //// Angular rates = 0
      //traj_goal_.jerk.x = 0;
      //traj_goal_.jerk.y = 0;
      //traj_goal_.jerk.z = 0;

      //// Publish the trajectory goal
      //// Note: the tracker will update the header.stamp and header.frame_id fields
      //pub_goal_trajectory_.publish(traj_goal_);
    }
    else
    {
      hover_in_place();
    }
  }
  */

  static tf::Matrix3x3 R;
  R.setEulerYPR(0, pitch, roll);
  R.getRotation(q);
  q.normalize();

#ifdef SAFETY_ON
  // Position and attitude Safety Catch
  if (safety && (abs(pos_.x) > 2.2 || abs(pos_.y) > 1.8|| pos_.z > 3.5 || pos_.z < 0.2))
  {
    ROS_WARN("Robot has exited safe box. Safety Catch initiated...");
    hover_in_place();
  }
#endif
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
  ROS_INFO("Quad using offsets: {xoff: %2.2f, yoff: %2.2f, zoff: %2.2f, yaw_off: %2.2f}", xoff, yoff, zoff, yaw_off);

  // n.param("state_control/traj_filename", traj_filename, string("traj.csv"));
  n.param("state_control/safety_catch", safety, true);
  n.param("mass", mass_, 0.5);

  // Publishers
  srv_transition_= n.serviceClient<controllers_manager::Transition>("controllers_manager/transition");
  pub_goal_min_jerk_ = n.advertise<geometry_msgs::Vector3>("controllers_manager/line_tracker_min_jerk/goal", 1);
  pub_goal_distance_ = n.advertise<geometry_msgs::Vector3>("controllers_manager/line_tracker_distance/goal", 1);
  pub_goal_velocity_ = n.advertise<quadrotor_msgs::FlatOutputs>("controllers_manager/velocity_tracker/vel_cmd_with_yaw", 1);
  pub_goal_yaw_ = n.advertise<quadrotor_msgs::FlatOutputs>("controllers_manager/line_tracker_yaw/goal", 1);
  // pub_goal_trajectory_ = n.advertise<quadrotor_msgs::PositionCommand>("controllers_manager/trajectory_tracker/goal", 1);
  // pub_info_bool_ = n.advertise<std_msgs::Bool>("traj_signal", 1);
  pub_motors_ = n.advertise<std_msgs::Bool>("motors", 1);
  pub_estop_ = n.advertise<std_msgs::Empty>("estop", 1);

  // Subscribers
  ros::Subscriber sub_odom = n.subscribe("odom", 1, &odom_cb, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub_imu = n.subscribe("imu", 1, &imu_cb, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub_nanokontrol = n.subscribe("/nanokontrol2", 1, nanokontrol_cb, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub_image_features = n.subscribe("image_features", 1, &image_update_cb, ros::TransportHints().tcpNoDelay());

  // Spin
  ros::spin();

  return 0;
}
