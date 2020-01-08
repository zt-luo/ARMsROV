#pragma once

#include <cstddef>
#include <cmath>
#include <thread>

#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Header.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/JointState.h>

#include <uuv_gazebo_ros_plugins_msgs/FloatStamped.h>

#include <tf/transform_datatypes.h>

#include "MavlinkHandler.h"

extern ros::NodeHandle* node;
