// Regular Includes
#include <Eigen/Dense>
#include <math.h>
#include <iostream>
#include <stdio.h>

// ROS Related Includes
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>

// Custom Includes
#include <controllers_manager/Transition.h>
#include <quadrotor_msgs/FlatOutputs.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/SO3Command.h>
#include <quadrotor_msgs/PWMCommand.h>
#include <quadrotor_msgs/OutputData.h>
#include "nano_kontrol2.h"
#include <trajectory.h>
#include <cylinder_msgs/ImageFeatures.h>
#include <cylinder_msgs/ParallelPlane.h>

using namespace std;
using namespace Eigen;
using namespace tf;

#define RED "\e[91m"
#define GREEN "\e[92m"
#define YELLOW "\e[93m"
#define BLUE "\e[94m"
#define MAGENTA "\e[95m"
#define CYAN "\e[96m"
#define RESET "\e[0m"

enum controller_state
{
  INIT,
  TAKEOFF,
  HOVER,
  LINE_TRACKER_MIN_JERK,
  LINE_TRACKER_YAW,
  LINE_TRACKER_DISTANCE,
  VELOCITY_TRACKER,
  VISION_CONTROL,
  RECOVERY,
  PREP_TRAJ,
  TRAJ,
  VISION_TRAJ,
  NONE,
  ESTOP,
};
// States
static enum controller_state state_ = INIT;

// Variables and parameters
static double xoff, yoff, zoff, yaw_off, mass_, gravity_, attitude_safety_limit_;
static bool safety_(true), safety_catch_active(false);
static geometry_msgs::Point home_;
std::string gstr;
sensor_msgs::Joy nk;
static bool nk_set(false);
double radio_channel_[8];
bool serial_;

// Stuff for trajectory
#include <string>
traj_type traj;
ros::Time traj_start_time;
double traj_time;
quadrotor_msgs::PositionCommand traj_goal_;
void updateTrajGoal();
static std::string traj_filename;
bool traj_loaded_(false), use_traj_gains_(false);

//#include <math.h>
// Publishers & services
static ros::Publisher pub_goal_min_jerk_;
static ros::Publisher pub_goal_distance_;
static ros::Publisher pub_goal_velocity_;
static ros::Publisher pub_motors_;
static ros::Publisher pub_estop_;
static ros::Publisher pub_goal_yaw_;
static ros::Publisher pub_traj_signal_;
static ros::Publisher pub_traj_goal_;
static ros::ServiceClient srv_transition_;
static ros::Publisher pub_vision_status_;
static ros::Publisher pub_so3_command_;
static ros::Publisher pub_pwm_command_;

static ros::Publisher pub_force_pos_, pub_force_vel_, pub_force_cmd_, pub_force_dyn_, pub_force_int_;

// Vision and Cylinder Stuff
// static tf::Transform T_Cam_to_Body_ = tf::Transform(tf::Matrix3x3(1,0,0, 0,-1,0, 0,0,-1), tf::Vector3(0,0,0));
static void Jacobians(const Vector3d &P1_inV, const Vector3d &sdot, const Matrix3d &R_VtoW, Matrix3d &Jinv, Matrix3d &Jdot);
double r;
double kR_[3], kOm_[3], corrections_[3];
bool enable_motors_, use_external_yaw_;
double kprho, kpu, kdrho, kdu, kirho, kiu;
double yaw_des_(0), yaw_des_dot_(0);
ros::Time last_image_update_;
int camera_rate;
double gains_mod_[4];
// Vector3d sdes_(-0.1, -0.1, 0.1);

// Quadrotor Pose
static geometry_msgs::Point pos_;
static geometry_msgs::Vector3 vel_;
static geometry_msgs::Quaternion ori_;
static tf::Quaternion imu_q_;
static double imu_yaw_;
static bool have_odom_(false), imu_info_ (false), vision_info_(false), need_odom_(true);
ros::Time last_odom_time_;

// Strings
static const std::string line_tracker_distance("line_tracker/LineTrackerDistance");
static const std::string line_tracker_min_jerk("line_tracker/LineTrackerMinJerk");
static const std::string line_tracker_yaw("line_tracker/LineTrackerYaw");
static const std::string velocity_tracker_str("velocity_tracker/VelocityTrackerYaw");
static const std::string null_tracker_str("null_tracker/NullTracker");

// Function Prototypes
void hover_in_place();
void hover_at(const geometry_msgs::Point goal);
void go_home();
void recovery();
void go_to(const quadrotor_msgs::FlatOutputs goal);

// Callbacks and functions
static void nanokontrol_cb(const sensor_msgs::Joy::ConstPtr &msg)
{
  nk_set = true;
  nk = *msg;

  // double rho_des = -0.39/2*(msg->axes[0]+1) - 0.01;
  // sdes_ = Vector3d(rho_des, rho_des, -0.8*(msg->axes[1]+1));

  if(msg->buttons[5])
  {
    quadrotor_msgs::PWMCommand pwm_cmd;
    pwm_cmd.pwm[0] = 0.1;
    pub_pwm_command_.publish(pwm_cmd);
  }
  else if (msg->buttons[6])
  {
    quadrotor_msgs::PWMCommand pwm_cmd;
    pwm_cmd.pwm[0] = 0.4;
    pub_pwm_command_.publish(pwm_cmd);
  }

  gains_mod_[0] = msg->axes[4]+1;
  gains_mod_[1] = msg->axes[5]+1;
  gains_mod_[2] = msg->axes[6]+1;
  gains_mod_[3] = msg->axes[7]+1;

  use_traj_gains_ = msg->axes[3] > 0;

  if(msg->buttons[estop_button])
  {
    state_ = ESTOP;

    // Publish the E-Stop command
    ROS_WARN("E-STOP");
    std_msgs::Empty estop_cmd;
    pub_estop_.publish(estop_cmd);

    // Disable motors
    ROS_WARN("Disarming motors...");
    std_msgs::Bool motors_cmd;
    motors_cmd.data = false;
    pub_motors_.publish(motors_cmd);

    // Switch to null_tracker so that the trackers do not publish so3_commands
    controllers_manager::Transition transition_cmd;
    transition_cmd.request.controller = null_tracker_str;
    srv_transition_.call(transition_cmd);

    // Create and publish the so3_command
    quadrotor_msgs::SO3Command::Ptr cmd(new quadrotor_msgs::SO3Command);
    // cmd->header.stamp = ros::Time::now();
    cmd->aux.enable_motors = false;
    pub_so3_command_.publish(cmd);
  }

  if (state_ == ESTOP)
    return;

  if(msg->buttons[7])
  {
    state_ = RECOVERY;
    recovery();
    return;
  }

  if(state_ == INIT)
  {
    if (need_odom_ && !have_odom_)
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
    else if(selected && msg->buttons[line_tracker_button] && (state_ == HOVER || state_ == LINE_TRACKER_MIN_JERK || state_ == TAKEOFF) && have_odom_)
    {
      state_ = LINE_TRACKER_MIN_JERK;
      ROS_INFO("Engaging controller: LINE_TRACKER_MIN_JERK");
      geometry_msgs::Point goal;
      goal.x = 2*msg->axes[0] + xoff;
      goal.y = 2*msg->axes[1] + yoff;
      goal.z = 2*msg->axes[2] + 2.0 + zoff;
      pub_goal_min_jerk_.publish(goal);
      controllers_manager::Transition transition_cmd;
      transition_cmd.request.controller = line_tracker_min_jerk;
      srv_transition_.call(transition_cmd);
    }
    // Line Tracker Yaw
    else if(selected && msg->buttons[line_tracker_yaw_button] && (state_ == HOVER || state_ == LINE_TRACKER_YAW || state_ == TAKEOFF) && have_odom_)
    {
      quadrotor_msgs::FlatOutputs goal;
      goal.x = 2*msg->axes[0] + xoff;
      goal.y = 2*msg->axes[1] + yoff;
      goal.z = 2*msg->axes[2] + 2.0 + zoff;
      goal.yaw = M_PI * msg->axes[3] + yaw_off;
      go_to(goal);
    }
    /*
    // Velocity Tracker
    else if(selected && msg->buttons[velocity_tracker_button] && state_ == HOVER && have_odom_)
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
    */

//    // Vision Control
//    else if(selected && msg->buttons[vision_control_button] && state_ == HOVER && vision_info_ && imu_info_)
//    {
//      ROS_INFO("Activating Vision Control.  state_ == VISION_CONTROL;");
//
//      controllers_manager::Transition transition_cmd;
//      transition_cmd.request.controller = null_tracker_str;
//      srv_transition_.call(transition_cmd);
//      state_ = VISION_CONTROL;
//    }
  }

  if(msg->buttons[traj_button] && (!need_odom_ || (state_ == HOVER && have_odom_)))
  {
    if (!traj_loaded_)
    {
      ROS_WARN("No trajectory loaded.  Not transitioning into PREP_TRAJ.");
      return;
    }
    else
    {
      state_ = PREP_TRAJ;
      ROS_INFO("Switching to vision control.  state_ == PREP_TRAJ;");

      // Updates traj goal to allow for correct initalization of the trajectory
      traj_start_time = ros::Time::now();
      updateTrajGoal();

      controllers_manager::Transition transition_cmd;
      transition_cmd.request.controller = null_tracker_str;
      srv_transition_.call(transition_cmd);
    }
  }
  else if(msg->buttons[play_button] && state_ == PREP_TRAJ)
  {
    // If we are ready to start the trajectory
    if (!need_odom_ || sqrt( pow(vel_.x,2) + pow(vel_.y,2) + pow(vel_.z,2) ) < 0.05)
    {
      ROS_INFO("Starting Trajectory");

      state_ = TRAJ;

      // Publish the trajectory signal
      std_msgs::Bool traj_on_signal;
      traj_on_signal.data = true;
      pub_traj_signal_.publish(traj_on_signal);

      traj_start_time = ros::Time::now();
      updateTrajGoal();

      controllers_manager::Transition transition_cmd;
      transition_cmd.request.controller = null_tracker_str;
      srv_transition_.call(transition_cmd);
    }
    else
    {
      ROS_WARN("Not ready to start trajectory.");
    }
  }
}

void updateTrajGoal()
{
  ros::Time current_time = ros::Time::now();
  ros::Duration delta_time = current_time - traj_start_time;
  traj_time = delta_time.toSec();

  unsigned long i = traj_time * 1000;

  if (i > traj.size()-1)
  {
    ROS_INFO_THROTTLE(1, "Trajectory completed.");

    std_msgs::Bool traj_signal;
    traj_signal.data = false;
    pub_traj_signal_.publish(traj_signal);

    i = traj.size() - 1;
  }

  // Note: traj[t_idx][flat_out][deriv]
  // Note: offsets have been removed for vision control
  traj_goal_.position.x = traj[i][0][0];
  traj_goal_.position.y = traj[i][1][0];
  traj_goal_.position.z = traj[i][2][0];

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

  pub_traj_goal_.publish(traj_goal_);

  if (traj[i].size() > 4)
  {
    traj_goal_.kx[0] = traj[i][4][0];
    traj_goal_.kx[1] = traj[i][4][1];
    traj_goal_.kx[2] = traj[i][4][2];
    traj_goal_.kv[0] = traj[i][4][3];
    traj_goal_.kv[1] = traj[i][4][4];
    traj_goal_.kv[2] = traj[i][4][5];
  }

  // ROS_INFO_THROTTLE(1, "Gains: kx: {%2.1f, %2.1f, %2.1f}, kv: {%2.1f, %2.1f, %2.1f}",
  //     traj[i][4][0],
  //     traj[i][4][1],
  //     traj[i][4][2],
  //     traj[i][4][3],
  //     traj[i][4][4],
  //     traj[i][4][5]);
}

static void imu_cb(const sensor_msgs::Imu::ConstPtr &msg)
{
  imu_info_ = true;
  tf::quaternionMsgToTF(msg->orientation, imu_q_);

  // Determine a geodesic angle from hover at the same yaw
  double roll, pitch, yaw;
  tf::Matrix3x3(imu_q_).getEulerYPR(yaw, pitch, roll);
  imu_yaw_ = yaw;

  // If we are currently executing a trajectory, update the setpoint
  if (state_ == TRAJ)
  {
    updateTrajGoal();
  }

  // Attitude safety catch
  if (safety_ && !safety_catch_active)
  {
    // Declarations
    tf::Quaternion q;
    tf::Matrix3x3 R;
    R.setEulerYPR(0, pitch, roll);
    double geodesic = std::fabs( std::acos(0.5 * (R[0][0] + R[1][1] + R[2][2] - 1)));

    if (geodesic > attitude_safety_limit_)
    {
      safety_catch_active = true;
      ROS_WARN("Robot attitude is %2.1f and has exceeded %2.1f degrees. Activating recovery...", geodesic*180/M_PI, attitude_safety_limit_*180/M_PI);
      recovery();
    }
  }

  // Image_update safety catch
  double frames_missed_limit = 10.0;
  if ((state_ == PREP_TRAJ || state_ == TRAJ || state_ == VISION_CONTROL)
      && (ros::Time::now() - last_image_update_).toSec() > frames_missed_limit * 1 / (double)camera_rate)
  {
    ROS_WARN("%2.0f image updates missed. Assuming tracking has been lost and activating recovery...", frames_missed_limit);
    recovery();
  }

  if (state_ != INIT && need_odom_ && (ros::Time::now() - last_odom_time_).toSec() > 0.2)
  {
    ROS_WARN("Odometry seems to have dropped. Attempting to recover...");
    recovery();
  }
}

static void output_data_cb(const quadrotor_msgs::OutputData::ConstPtr &msg)
{
  for (unsigned int i = 0; i < 8; i++)
    radio_channel_[i] = msg->radio_channel[i];

  serial_ = radio_channel_[4] > 0;

  static controller_state last_state = NONE;
  if (state_ == RECOVERY)
  {
    double rc_max_angle = 10.0 * M_PI / 180.0;
    double scale = 255.0 / 2.0;
    double roll  = (radio_channel_[1] / scale - 1.0) * rc_max_angle;
    double pitch = - (radio_channel_[0] / scale - 1.0) * rc_max_angle;

    double rc_max_yawrate = 50.0 * M_PI / 180.0;
    static double yaw = imu_yaw_;
    yaw = last_state == RECOVERY ? yaw : imu_yaw_;
    yaw += -(radio_channel_[3] / scale - 1.0) * rc_max_yawrate / 100.0 ;
    Eigen::Quaterniond q(AngleAxisd(yaw, Vector3d::UnitZ()) * AngleAxisd(pitch, Vector3d::UnitY()) * AngleAxisd(roll,  Vector3d::UnitX()));

    // Create and publish the so3_command
    quadrotor_msgs::SO3Command::Ptr cmd(new quadrotor_msgs::SO3Command);
    cmd->header.stamp = ros::Time::now();
    cmd->force.x = 0;
    cmd->force.y = 0;
    cmd->force.z = 0.9 * mass_ * gravity_;

    cmd->orientation.x = q.x();
    cmd->orientation.y = q.y();
    cmd->orientation.z = q.z();
    cmd->orientation.w = q.w();
    cmd->angular_velocity.x = 0;
    cmd->angular_velocity.y = 0;
    cmd->angular_velocity.z = 0;
    for(int i = 0; i < 3; i++)
    {
      cmd->kR[i] = kR_[i];
      cmd->kOm[i] = kOm_[i];
    }
    // cmd->aux.current_yaw = 0;
    cmd->aux.kf_correction = corrections_[0];
    cmd->aux.angle_corrections[0] = corrections_[1];
    cmd->aux.angle_corrections[1] = corrections_[2];
    cmd->aux.enable_motors = true;
    cmd->aux.use_external_yaw = false;

    pub_so3_command_.publish(cmd);
  }
  last_state = state_;

  // if (!nk_set)
  // {
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
  // }
}

static void image_update_cb(const cylinder_msgs::ParallelPlane::ConstPtr &msg)
{
  vision_info_ = true;

  Eigen::Matrix3d R_VtoW(Eigen::Quaterniond(msg->qVtoW.w, msg->qVtoW.x, msg->qVtoW.y, msg->qVtoW.z));

  tf::Quaternion qBtoW(msg->qBtoW.x, msg->qBtoW.y, msg->qBtoW.z, msg->qBtoW.w);
  double current_yaw = tf::getYaw(qBtoW);

  Vector3d P1(msg->P1.x, msg->P1.y, msg->P1.z);
  Vector3d s = Vector3d(msg->s[0], msg->s[1], msg->s[2]);
  Vector3d sdot = Vector3d(msg->sdot[0], msg->sdot[1], msg->sdot[2]);

  Matrix3d Jinv, Jdot;

  if (isnan(P1(0)) || isnan(P1(1)) || isnan(P1(2)))
    ROS_WARN("P1 is nan");
  if (isnan(sdot(0)) || isnan(sdot(1)) || isnan(sdot(2)))
    ROS_WARN("sdot is nan");
  if (isnan(Vector3d(1,1,1).transpose() * R_VtoW * Vector3d(1,1,1)))
    ROS_WARN("R_VtoW is nan");

  Jacobians(P1, sdot, R_VtoW, Jinv, Jdot);

  // Matrix3d T = R_VtoW * Jinv;
  // cout << CYAN << "Jinv = " << endl << Jinv << RESET << endl; // cout << BLUE << "T = " << endl << T << RESET << endl;

  Vector3d sdes(traj_goal_.position.x, traj_goal_.position.y, traj_goal_.position.z);
  Vector3d sdotdes(traj_goal_.velocity.x, traj_goal_.velocity.y, traj_goal_.velocity.z),
           sddotdes(traj_goal_.acceleration.x, traj_goal_.acceleration.y, traj_goal_.acceleration.z),
           sdddotdes(traj_goal_.jerk.x, traj_goal_.jerk.y, traj_goal_.jerk.z);

  // For now...
  // static Vector3d sdes = s;

  yaw_des_ = traj_goal_.yaw;
  yaw_des_dot_ = traj_goal_.yaw_dot;

  // Useful defs
  static const Vector3d e3(0,0,1);

  // Errors
  Vector3d e_pos(sdes - s);
  Vector3d e_vel(sdotdes - sdot);

  // In case the trajectory goal has not been set
  if (isnan(e_pos(0)) || isnan(e_pos(1)) || isnan(e_pos(2))) e_pos = Vector3d(0,0,0);

  ROS_INFO_THROTTLE(1, MAGENTA "s    = {%2.2f, %2.2f, %2.2f}" RESET, s(0), s(1), s(2));
  ROS_INFO_THROTTLE(1, MAGENTA "sdes = {%2.2f, %2.2f, %2.2f}" RESET, sdes(0), sdes(1), sdes(2));
  // ROS_INFO_THROTTLE(1, "\e[91me_pos: {%2.2f, %2.2f, %2.2f}\e[0m", e_pos(0), e_pos(1), e_pos(2));
  // ROS_INFO_THROTTLE(1, "\e[93me_vel: {%2.2f, %2.2f, %2.2f}\e[0m", e_vel(0), e_vel(1), e_vel(2));

  // Gains
  Eigen::Vector3d ki(kirho, kirho, kiu);

  Vector3d kx, kv;
  if (use_traj_gains_)
  {
    kx << traj_goal_.kx[0], traj_goal_.kx[1], traj_goal_.kx[2];
    kv << traj_goal_.kv[0], traj_goal_.kv[1], traj_goal_.kv[2];
    ROS_INFO_THROTTLE(1, BLUE "Traj Gains: {kprho, kdrho, kpu, kdu, kirho, kiu}: {%2.2f, %2.2f, %2.2f, %2.2f, %2.2f, %2.2f}" RESET, kx(0), kv(0), kx(2), kv(2), kirho, kiu);
  }
  else
  {
    kx << kprho * gains_mod_[0], kprho * gains_mod_[0], kpu * gains_mod_[2];
    kv << kdrho * gains_mod_[1], kdrho * gains_mod_[1], kdu * gains_mod_[3];
    ROS_INFO_THROTTLE(1, BLUE "Midi Gains: {kprho, kdrho, kpu, kdu, kirho, kiu}: {%2.2f, %2.2f, %2.2f, %2.2f, %2.2f, %2.2f}" RESET, kx(0), kv(0), kx(2), kv(2), kirho, kiu);
  }

  // Temp output
  // Vector3d temp = mass_ * R_VtoW * Jinv * (kx.asDiagonal() * e_pos); //  + kv.asDiagonal() * e_vel + sddotdes);
  // ROS_INFO_THROTTLE(1, "\e[93mdelta_force_in_pp: {%2.2f, %2.2f, %2.2f}\e[0m", temp(0), temp(1), temp(2));

  if (isnan( Vector3d(1,1,1).transpose() * Jinv * Vector3d(1,1,1)))
    ROS_WARN("Jinv is nan!");

  static Eigen::Vector3d fint(-0.02, -0.13, 0.0);
  // static Eigen::Vector3d fint(0, 0, 0);
  fint += mass_ * Jinv * ki.asDiagonal() * e_pos;

  ROS_INFO_THROTTLE(1, "fint = {%2.2f, %2.2f, %2.2f}", fint(0), fint(1), fint(2));

  Matrix3d Jinvdot = - Jinv * Jdot * Jinv;

  // Nominal thrust (in the world)
  Vector3d force = mass_* (gravity_ * e3
    + Jinv * kx.asDiagonal() * e_pos
    + Jinv * kv.asDiagonal() * e_vel
    + Jinv * sddotdes
    + Jinvdot * sdot)
    + fint;

  // For now, reduce the thrust magnitude
  // force = fmin(force.norm(), 0.95 * mass_ * g) * force.normalized();

  ROS_INFO_THROTTLE(1, GREEN "force/weight {x, y, z} = {%2.4f, %2.4f, %2.4f}, gains mod = {%1.2f, %1.2f, %1.2f, %1.2f}" RESET,
      force(0) / (mass_*gravity_), force(1) / (mass_*gravity_), force(2) / (mass_*gravity_), gains_mod_[0], gains_mod_[1], gains_mod_[2], gains_mod_[3]);


  //============================//
  //==== Publish Force Info ====//
  //============================//

  geometry_msgs::Vector3 temp;

  temp.x = force(0); temp.y = force(1); temp.z = force(2);
  pub_force_cmd_.publish(temp);

  Vector3d force1 = mass_ * Jinv * kx.asDiagonal() * e_pos;
  temp.x = force1(0); temp.y = force1(1); temp.z = force1(2);
  pub_force_pos_.publish(temp);
  // ROS_INFO_THROTTLE(1, GREEN "Position component of force: {%2.2f, %2.2f, %2.2f}" RESET, force1(0), force1(1), force1(2));

  Vector3d force2 = mass_ * Jinv * kv.asDiagonal() * e_vel;
  temp.x = force2(0); temp.y = force2(1); temp.z = force2(2);
  pub_force_vel_.publish(temp);
  // ROS_INFO_THROTTLE(1, RED "Velocity component of force: {%2.2f, %2.2f, %2.2f}" RESET, force2(0), force2(1), force2(2));

  Vector3d force3 = mass_ * Jinvdot * sdot;
  temp.x = force3(0); temp.y = force3(1); temp.z = force3(2);
  pub_force_dyn_.publish(temp);

  temp.x = fint(0); temp.y = fint(1); temp.z = fint(2);
  pub_force_int_.publish(temp);

  //============================//
  //============================//


  Eigen::Vector3d b1c, b2c, b3c;
  Eigen::Vector3d b1d(cos(yaw_des_), sin(yaw_des_), 0);

  if(force.norm() > 1e-6)
    b3c.noalias() = force.normalized();
  else
    b3c.noalias() = Eigen::Vector3d(0, 0, 1);

  b2c.noalias() = b3c.cross(b1d).normalized();
  b1c.noalias() = b2c.cross(b3c).normalized();

  // Determine angular velocity?

  Eigen::Matrix3d Rc;
  Rc << b1c, b2c, b3c;
  Eigen::Quaterniond qdes(Rc);

  //Eigen::Matrix3d R_dot;
  //R_dot << b1c_dot, b2c_dot, b3c_dot;

  //const Eigen::Matrix3d omega_hat = Rc.transpose() * R_dot;
  //Eigen::Vector3d angular_velocity = Eigen::Vector3d(omega_hat(2,1), omega_hat(0,2), omega_hat(1,0));

  // Only publish if we are in the vision control state
  if (state_ == PREP_TRAJ || state_ == TRAJ || state_ == VISION_CONTROL)
  {
    quadrotor_msgs::SO3Command::Ptr cmd(new quadrotor_msgs::SO3Command);
    cmd->header.stamp = msg->stamp;
    // cmd->header.frame_id = 0;
    cmd->force.x = force(0);
    cmd->force.y = force(1);
    cmd->force.z = force(2);
    cmd->orientation.x = qdes.x();
    cmd->orientation.y = qdes.y();
    cmd->orientation.z = qdes.z();
    cmd->orientation.w = qdes.w();
    cmd->angular_velocity.x = 0; // *angular_velocity(0);
    cmd->angular_velocity.y = 0; // *angular_velocity(1);
    cmd->angular_velocity.z = 0; // *angular_velocity(2);
    for(int i = 0; i < 3; i++)
    {
      cmd->kR[i] = kR_[i];
      cmd->kOm[i] = kOm_[i];
    }
    cmd->aux.current_yaw = current_yaw;
    cmd->aux.kf_correction = corrections_[0];
    cmd->aux.angle_corrections[0] = corrections_[1];
    cmd->aux.angle_corrections[1] = corrections_[2];
    cmd->aux.enable_motors = true;
    cmd->aux.use_external_yaw = true; // use_external_yaw_;

    pub_so3_command_.publish(cmd);
    ROS_INFO_THROTTLE(1, "Vision control");
  }

  last_image_update_ = ros::Time::now();
}

void Jacobians(const Vector3d &P1_inV, const Vector3d &sdot, const Matrix3d &R_VtoW, Matrix3d &Jinv, Matrix3d &Jdot)
{
  // These values are correct in the virtual frame
  // ROS_INFO_THROTTLE(1, "\e[94mUsing {x1, y1, z1, r} = {%2.2f, %2.2f, %2.2f, %2.2f} in V\e[0m", P1_inV(0), P1_inV(1), P1_inV(2), r);

  double x1, y1, z1;
  x1 = P1_inV(0);
  y1 = P1_inV(1);
  z1 = P1_inV(2);

  // ROS_INFO_THROTTLE(1, "\e[93mUsing {x1, y1, z1, r} = {%2.2f, %2.2f, %2.2f, %2.2f} to compute J\e[0m", x1, y1, z1, r);
  Matrix3d J;

  J(0,0) = 0;
  J(0,1) = (pow(y1,2) + pow(z1,2))/((-pow(r,2) + pow(y1,2))*z1 + pow(z1,3) - r*y1*sqrt(-pow(r,2) + pow(y1,2) + pow(z1,2)));
  J(0,2) = -(((pow(y1,2) + pow(z1,2))*(r*z1 + y1*sqrt(-pow(r,2) + pow(y1,2) + pow(z1,2))))/(sqrt(-pow(r,2) + pow(y1,2) + pow(z1,2))*pow(r*y1 - z1*sqrt(-pow(r,2) + pow(y1,2) + pow(z1,2)),2)));
  J(1,0) = 0;
  J(1,1) = (pow(y1,2) + pow(z1,2))/((-pow(r,2) + pow(y1,2))*z1 + pow(z1,3) + r*y1*sqrt(-pow(r,2) + pow(y1,2) + pow(z1,2)));
  J(1,2) = -(((pow(y1,2) + pow(z1,2))*(-(r*z1) + y1*sqrt(-pow(r,2) + pow(y1,2) + pow(z1,2))))/(sqrt(-pow(r,2) + pow(y1,2) + pow(z1,2))*pow(r*y1 + z1*sqrt(-pow(r,2) + pow(y1,2) + pow(z1,2)),2)));
  J(2,0) = 1/z1;
  J(2,1) = 0;
  J(2,2) = -(x1/pow(z1,2));

  // cout << CYAN << "J = " << endl << J << RESET << endl;
  // cout << MAGENTA << "{x1, y1, z1, r} = {" << x1 << ", " << y1 << ", " << z1 << ", " << r << "}" << endl;

  // Estimate the velocity of p1 in the camera frame
  Vector3d p1dot = J.inverse() * sdot;
  double x1dot, y1dot, z1dot;
  x1dot = p1dot(0);
  y1dot = p1dot(1);
  z1dot = p1dot(2);

  Jdot(0,0) = 0;
  Jdot(0,1) = (2*((-pow(r,2) + pow(y1,2))*z1 + pow(z1,3) - r*y1*sqrt(-pow(r,2) + pow(y1,2) + pow(z1,2)))*(y1*y1dot + z1*z1dot) - (pow(y1,2) + pow(z1,2))*(2*y1*z1*y1dot - r*sqrt(-pow(r,2) + pow(y1,2) + pow(z1,2))*y1dot + (-pow(r,2) + pow(y1,2))*z1dot + 3*pow(z1,2)*z1dot - (r*y1*(y1*y1dot + z1*z1dot))/sqrt(-pow(r,2) + pow(y1,2) + pow(z1,2))))/pow((-pow(r,2) + pow(y1,2))*z1 + pow(z1,3) - r*y1*sqrt(-pow(r,2) + pow(y1,2) + pow(z1,2)),2);
  Jdot(0,2) = (-(pow(y1,6)*z1*y1dot) + 2*pow(y1,7)*z1dot + pow(y1,5)*(r*sqrt(-pow(r,2) + pow(y1,2) + pow(z1,2))*y1dot - 3*(pow(r,2) - 2*pow(z1,2))*z1dot) + 3*pow(y1,4)*z1*((pow(r,2) - pow(z1,2))*y1dot + r*sqrt(-pow(r,2) + pow(y1,2) + pow(z1,2))*z1dot) + pow(z1,3)*((pow(r,4) - pow(z1,4))*y1dot + r*sqrt(-pow(r,2) + pow(y1,2) + pow(z1,2))*(pow(r,2) + 2*pow(z1,2))*z1dot) + pow(y1,2)*z1*(-3*(pow(r,4) - pow(r,2)*pow(z1,2) + pow(z1,4))*y1dot + r*sqrt(-pow(r,2) + pow(y1,2) + pow(z1,2))*(-3*pow(r,2) + 5*pow(z1,2))*z1dot) + y1*pow(z1,2)*(3*pow(r,3)*sqrt(-pow(r,2) + pow(y1,2) + pow(z1,2))*y1dot + (-3*pow(r,4) + 2*pow(z1,4))*z1dot) + pow(y1,3)*(r*(-pow(r,2) + pow(z1,2))*sqrt(-pow(r,2) + pow(y1,2) + pow(z1,2))*y1dot + (pow(r,4) - 3*pow(r,2)*pow(z1,2) + 6*pow(z1,4))*z1dot))/(pow(-pow(r,2) + pow(y1,2) + pow(z1,2),1.5)*pow(-(r*y1) + z1*sqrt(-pow(r,2) + pow(y1,2) + pow(z1,2)),3));
  Jdot(1,0) = 0;
  Jdot(1,1) = (2*((-pow(r,2) + pow(y1,2))*z1 + pow(z1,3) + r*y1*sqrt(-pow(r,2) + pow(y1,2) + pow(z1,2)))*(y1*y1dot + z1*z1dot) - (pow(y1,2) + pow(z1,2))*(2*y1*z1*y1dot + r*sqrt(-pow(r,2) + pow(y1,2) + pow(z1,2))*y1dot + (-pow(r,2) + pow(y1,2))*z1dot + 3*pow(z1,2)*z1dot + (r*y1*(y1*y1dot + z1*z1dot))/sqrt(-pow(r,2) + pow(y1,2) + pow(z1,2))))/pow((-pow(r,2) + pow(y1,2))*z1 + pow(z1,3) + r*y1*sqrt(-pow(r,2) + pow(y1,2) + pow(z1,2)),2);
  Jdot(1,2) = -((pow(y1,6)*z1*y1dot - 2*pow(y1,7)*z1dot + pow(y1,5)*(r*sqrt(-pow(r,2) + pow(y1,2) + pow(z1,2))*y1dot + 3*(pow(r,2) - 2*pow(z1,2))*z1dot) + 3*pow(y1,4)*z1*((-pow(r,2) + pow(z1,2))*y1dot + r*sqrt(-pow(r,2) + pow(y1,2) + pow(z1,2))*z1dot) + pow(z1,3)*((-pow(r,4) + pow(z1,4))*y1dot + r*sqrt(-pow(r,2) + pow(y1,2) + pow(z1,2))*(pow(r,2) + 2*pow(z1,2))*z1dot) + pow(y1,2)*z1*(3*(pow(r,4) - pow(r,2)*pow(z1,2) + pow(z1,4))*y1dot + r*sqrt(-pow(r,2) + pow(y1,2) + pow(z1,2))*(-3*pow(r,2) + 5*pow(z1,2))*z1dot) + y1*pow(z1,2)*(3*pow(r,3)*sqrt(-pow(r,2) + pow(y1,2) + pow(z1,2))*y1dot + (3*pow(r,4) - 2*pow(z1,4))*z1dot) - pow(y1,3)*(r*(pow(r,2) - pow(z1,2))*sqrt(-pow(r,2) + pow(y1,2) + pow(z1,2))*y1dot + (pow(r,4) - 3*pow(r,2)*pow(z1,2) + 6*pow(z1,4))*z1dot))/(pow(-pow(r,2) + pow(y1,2) + pow(z1,2),1.5)*pow(r*y1 + z1*sqrt(-pow(r,2) + pow(y1,2) + pow(z1,2)),3)));
  Jdot(2,0) = -(z1dot/pow(z1,2));
  Jdot(2,1) = 0;
  Jdot(2,2) = (-(z1*x1dot) + 2*x1*z1dot)/pow(z1,3);

  // Now, apply rotations to the world frame
  // Note: The minus sign gives us the velocity of the robot relative to P1
  // instead of the velocity of P1 relative to the robot
  J = - J * R_VtoW.transpose();
  Jdot = - Jdot * R_VtoW.transpose();

  // cout << "Det(J): " << J.determinant() << endl;
  Jinv = J.inverse();
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

void recovery()
{
  // We do not want to re-enable the motors if the ESTOP has been triggered
  if (state_ == ESTOP)
    return;

  if (state_ == INIT || state_ == NONE || state_ == ESTOP)
    return;

  state_ = RECOVERY;

  if (have_odom_ && (ros::Time::now() - last_odom_time_).toSec() < 0.1)
    hover_in_place();
  else
  {
    ROS_WARN("Recovery without odometry...");

    // Switch to null_tracker so that the trackers do not publish so3_commands
    controllers_manager::Transition transition_cmd;
    transition_cmd.request.controller = null_tracker_str;
    srv_transition_.call(transition_cmd);

    // Create and publish the so3_command
    quadrotor_msgs::SO3Command::Ptr cmd(new quadrotor_msgs::SO3Command);
    cmd->header.stamp = ros::Time::now();
    // cmd->header.frame_id = 0;
    cmd->force.x = 0;
    cmd->force.y = 0;
    cmd->force.z = 0.9 * mass_ * gravity_;

    cmd->orientation.x = 0;
    cmd->orientation.y = 0;
    cmd->orientation.z = 0;
    cmd->orientation.w = 1;
    cmd->angular_velocity.x = 0;
    cmd->angular_velocity.y = 0;
    cmd->angular_velocity.z = 0;
    for(int i = 0; i < 3; i++)
    {
      cmd->kR[i] = kR_[i];
      cmd->kOm[i] = kOm_[i];
    }
    cmd->aux.current_yaw = 0;
    cmd->aux.kf_correction = corrections_[0];
    cmd->aux.angle_corrections[0] = corrections_[1];
    cmd->aux.angle_corrections[1] = corrections_[2];
    cmd->aux.enable_motors = true;
    cmd->aux.use_external_yaw = true;

    pub_so3_command_.publish(cmd);
  }
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
  last_odom_time_ = ros::Time::now();
  have_odom_ = true;

  pos_ = msg->pose.pose.position;
  vel_ = msg->twist.twist.linear;
  ori_ = msg->pose.pose.orientation;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "state_control");
  ros::NodeHandle n("~");

  // Load params

  // Offsets for this robot
  n.param("offsets/x", xoff, 0.0);
  n.param("offsets/y", yoff, 0.0);
  n.param("offsets/z", zoff, 0.0);
  n.param("offsets/yaw", yaw_off, 0.0);
  ROS_INFO("Quad using offsets: {xoff: %2.2f, yoff: %2.2f, zoff: %2.2f, yaw_off: %2.2f}", xoff, yoff, zoff, yaw_off);

  // Safety
  n.param("need_odom", need_odom_, true);
  if (!need_odom_)
    ROS_WARN("Not using odometry...");

  n.param("safety_catch", safety_, true);
  if (!safety_)
    ROS_WARN("Safety catch is off!");

  // Properties
  n.param("gravity", gravity_, 9.81);
  n.param("mass", mass_, 0.5);
  ROS_INFO("State_control using mass = %2.2f", mass_);

  // Params needed for so3 control from vision
  n.param("use_external_yaw", use_external_yaw_, true);

  // Attitude gains
  n.param("gains/rot/x", kR_[0], 1.5);
  n.param("gains/rot/y", kR_[1], 1.5);
  n.param("gains/rot/z", kR_[2], 1.0);
  n.param("gains/ang/x", kOm_[0], 0.13);
  n.param("gains/ang/y", kOm_[1], 0.13);
  n.param("gains/ang/z", kOm_[2], 0.1);
  ROS_INFO("Attitude gains: kR: {%2.2f, %2.2f, %2.2f}, kOm: {%2.2f, %2.2f, %2.2f}", kR_[0], kR_[1], kR_[2], kOm_[0], kOm_[1], kOm_[2]);
  n.param("attitude_safety_limit", attitude_safety_limit_, 25 * M_PI / 180);

  // Corrections
  n.param("corrections/kf", corrections_[0], 0.0);
  n.param("corrections/r", corrections_[1], 0.0);
  n.param("corrections/p", corrections_[2], 0.0);

  /////////////
  // Vision //
  ///////////

  // Radius
  n.param("/cylinder_radius", r, 0.1);
  ROS_INFO("State control using Cylinder Radius = %2.2f", r);

  // Vision gains
  n.param("vision_gains/kprho", kprho, 0.0);
  n.param("vision_gains/kdrho", kdrho, 0.0);
  n.param("vision_gains/kpu", kpu, 0.0);
  n.param("vision_gains/kdu", kdu, 0.0);
  n.param("vision_gains/kirho", kirho, 0.0);
  n.param("vision_gains/kiu", kiu, 0.0);
  n.param("camera_fps", camera_rate, 50);
  // ROS_INFO("Vision using gains: {kprho: %2.2f, kpu: %2.2f, kdrho: %2.2f, kdu: %2.2f}", kprho, kpu, kdrho, kdu);

  n.param("traj_filename", traj_filename, string(""));
  // Note: traj[t_idx][flat_out][deriv]
  int flag = loadTraj(traj_filename.c_str(), traj);
  if (flag == 0)
  {
    ROS_INFO(GREEN "Successfully loaded: traj_filename: \"%s\"" RESET, traj_filename.c_str());
    traj_loaded_ = true;
  }
  else
    ROS_WARN("Couldn't load trajectory: %s.  Error: %d.", traj_filename.c_str(), flag);

  // Timing stuff
  last_image_update_ = ros::Time::now();
  last_odom_time_ = ros::Time::now();

  /////////////////
  // Publishers //
  ///////////////

  srv_transition_= n.serviceClient<controllers_manager::Transition>("controllers_manager/transition");
  pub_goal_min_jerk_ = n.advertise<geometry_msgs::Vector3>("min_jerk_goal", 10);
  pub_goal_distance_ = n.advertise<geometry_msgs::Vector3>("line_tracker_distance_goal", 10);
  pub_goal_velocity_ = n.advertise<quadrotor_msgs::FlatOutputs>("vel_goal", 10);
  pub_goal_yaw_ = n.advertise<quadrotor_msgs::FlatOutputs>("line_tracker_yaw_goal", 10);
  pub_traj_signal_ = n.advertise<std_msgs::Bool>("traj_signal", 1);
  pub_traj_goal_ = n.advertise<quadrotor_msgs::PositionCommand>("traj_goal", 10);
  pub_motors_ = n.advertise<std_msgs::Bool>("motors", 1);
  pub_estop_ = n.advertise<std_msgs::Empty>("estop", 1);
  pub_so3_command_ = n.advertise<quadrotor_msgs::SO3Command>("so3_cmd", 1);
  pub_pwm_command_ = n.advertise<quadrotor_msgs::PWMCommand>("pwm_cmd", 1);

  pub_force_pos_ = n.advertise<geometry_msgs::Vector3>("force/pos", 1);
  pub_force_vel_ = n.advertise<geometry_msgs::Vector3>("force/vel", 1);
  pub_force_cmd_ = n.advertise<geometry_msgs::Vector3>("force/cmd", 1);
  pub_force_dyn_ = n.advertise<geometry_msgs::Vector3>("force/dyn", 1);
  pub_force_int_ = n.advertise<geometry_msgs::Vector3>("force/int", 1);

  // Subscribers
  ros::Subscriber sub_odom = n.subscribe("odom", 1, &odom_cb, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub_imu = n.subscribe("imu", 1, &imu_cb, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub_nanokontrol = n.subscribe("/nanokontrol2", 1, nanokontrol_cb, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub_vision = n.subscribe("image_features_pp", 1, &image_update_cb, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub_output = n.subscribe("output_data", 1, &output_data_cb, ros::TransportHints().tcpNoDelay());

  // Switch to null_tracker so that the trackers do not publish so3_commands
  controllers_manager::Transition transition_cmd;
  transition_cmd.request.controller = null_tracker_str;
  srv_transition_.call(transition_cmd);

  // Create and publish the so3_command
  quadrotor_msgs::SO3Command::Ptr cmd(new quadrotor_msgs::SO3Command);
  cmd->aux.enable_motors = false;
  pub_so3_command_.publish(cmd);

  // Spin
  ros::spin();

  return 0;
}
