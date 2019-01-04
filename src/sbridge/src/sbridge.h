#ifndef SBRIDGE
#define SBRIDGE

// ROS messages
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt8.h>

#include <swarmie_msgs/Skid.h>

using namespace std;

/**
 * This class translates drive controls into Gazebo
 * friendly velocities.
 */
class sbridge {

public:

  sbridge(std::string published_name);
  void cmdHandler(const swarmie_msgs::Skid::ConstPtr& message);
  ~sbridge();

private:

  // Publishers
  ros::Publisher skidsteer_publisher;
  ros::Publisher heartbeat_publisher;
  ros::Publisher info_log_publisher;

  // Subscribers
  ros::Subscriber drive_control_subscriber;
  ros::Subscriber mode_subscriber;

  // Timer callback handler
  void publishHeartBeatTimerEventHandler(const ros::TimerEvent& event);

  ros::Timer publish_heartbeat_timer;

  geometry_msgs::Twist velocity;
};

#endif /* SBRIDGE */
