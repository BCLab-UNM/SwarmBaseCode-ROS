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

using namespace std;

/**
 * This class translates drive controls into Gazebo
 * friendly velocities.
 */
class sbridge {

	public:

		sbridge(std::string publishedName);
		void cmdHandler(const geometry_msgs::Twist::ConstPtr& message);
        ~sbridge();

	private:

                // Publishers
		ros::Publisher skidsteerPublish;
        ros::Publisher heartbeatPublisher;
        ros::Publisher infoLogPublisher;

                // Subscribers
		ros::Subscriber driveControlSubscriber;
        ros::Subscriber modeSubscriber;

        // Timer callback handler
        void publishHeartBeatTimerEventHandler(const ros::TimerEvent& event);

        ros::Timer publish_heartbeat_timer;

		geometry_msgs::Twist velocity;
};

#endif /* SBRIDGE */
