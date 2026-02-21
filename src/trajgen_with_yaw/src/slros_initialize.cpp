#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "trajGen_with_yaw";

// For Block trajGen_with_yaw/Subscribe
SimulinkSubscriber<geometry_msgs::Pose, SL_Bus_trajGen_with_yaw_geometry_msgs_Pose> Sub_trajGen_with_yaw_321;

// For Block trajGen_with_yaw/Subscribe1
SimulinkSubscriber<std_msgs::Bool, SL_Bus_trajGen_with_yaw_std_msgs_Bool> Sub_trajGen_with_yaw_322;

// For Block trajGen_with_yaw/position reading
SimulinkSubscriber<nav_msgs::Odometry, SL_Bus_trajGen_with_yaw_nav_msgs_Odometry> Sub_trajGen_with_yaw_377;

// For Block trajGen_with_yaw/Publish
SimulinkPublisher<geometry_msgs::Twist, SL_Bus_trajGen_with_yaw_geometry_msgs_Twist> Pub_trajGen_with_yaw_319;

// For Block trajGen_with_yaw/Get Parameter test
SimulinkParameterGetter<real64_T, double> ParamGet_trajGen_with_yaw_317;

void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}

