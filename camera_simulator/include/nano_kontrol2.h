#ifndef NANO_KONTROL2
#define NANO_KONTROL2

#include <sensor_msgs/Joy.h>

// Buttons
static const int vision_control_button = 25; // Cycle
static const int traj_button = 34; // Cycle
static const int estop_button = 26;  // Stop
static const int play_button = 27;  // Play
static const int motors_on_button = 28;  // Rec
static const int line_tracker_button = 29;  // Track L
static const int velocity_tracker_button = 30; // Track R
static const int hover_button = 31; // Marker Set
static const int line_tracker_yaw_button = 24; // Rewind 

#endif
