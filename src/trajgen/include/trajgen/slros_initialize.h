#ifndef _SLROS_INITIALIZE_H_
#define _SLROS_INITIALIZE_H_

#include "slros_busmsg_conversion.h"
#include "slros_generic.h"
#include "trajGen_types.h"

extern ros::NodeHandle * SLROSNodePtr;
extern const std::string SLROSNodeName;

// For Block trajGen/Subscribe
extern SimulinkSubscriber<geometry_msgs::Point, SL_Bus_trajGen_geometry_msgs_Point> Sub_trajGen_321;

// For Block trajGen/Subscribe1
extern SimulinkSubscriber<std_msgs::Bool, SL_Bus_trajGen_std_msgs_Bool> Sub_trajGen_322;

// For Block trajGen/position reading
extern SimulinkSubscriber<nav_msgs::Odometry, SL_Bus_trajGen_nav_msgs_Odometry> Sub_trajGen_377;

// For Block trajGen/Publish
extern SimulinkPublisher<geometry_msgs::Twist, SL_Bus_trajGen_geometry_msgs_Twist> Pub_trajGen_319;

// For Block trajGen/Publish1
extern SimulinkPublisher<std_msgs::Float64, SL_Bus_trajGen_std_msgs_Float64> Pub_trajGen_378;

// For Block trajGen/Get Parameter
extern SimulinkParameterGetter<real64_T, double> ParamGet_trajGen_317;

void slros_node_init(int argc, char** argv);

#endif
