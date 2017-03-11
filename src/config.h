#pragma once

#include "landmark.h"

// All metric units are in pixels.

// world settings
static const int WORLD_WIDTH = 1000;
static const int WORLD_HEIGHT= 1000;

static const int BOUNDARY_X1 = 50;
static const int BOUNDARY_X2 = 950;
static const int BOUNDARY_Y1 = 50;
static const int BOUNDARY_Y2 = 950;

// landmarks are randomly generated
static const int TOTAL_LANDMARKS = 20;

// motion noise parameters
// velocity_noise          = alpha1*|velocity| + alpha2*|angular_velocity|
// angular_velocity_noise  = alpha3*|velocity| + alpha4*|angular_velocity|
static const double ALPHA1 = 0.1;
static const double ALPHA2 = 0;
static const double ALPHA3 = 0.001;
static const double ALPHA4 = 0.1;

// robot motion properties
static const double ROBOT_VEL = 300; // pixels/sec
static const double ROBOT_YAW_VEL = 120*M_PI/180; // rad/sec

// properties of the robot's sensor
static const double FOV = 45*M_PI/180;
static const double DETECTION_RANGE = 200;
static const double DETECTION_RANGE_ALPHA = 0.1; // detection error is a percentage of the range
static const double DETECTION_ANGLE_SIGMA = 2*M_PI/180;

// 99% confidence ellipse for 2 dimensions
static const double ELLIPSE_SCALE = 3.0348;
