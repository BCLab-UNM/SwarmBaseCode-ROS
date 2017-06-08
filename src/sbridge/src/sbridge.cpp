#include "sbridge.h"

sbridge::sbridge(std::string publishedName) {
    ros::NodeHandle param("~");
    ros::NodeHandle sNH;

    driveControlSubscriber = sNH.subscribe((publishedName + "/driveControl"), 10, &sbridge::cmdHandler, this);

    heartbeatPublisher = sNH.advertise<std_msgs::String>((publishedName + "/sbridge/heartbeat"), 1, false);
    skidsteerPublish = sNH.advertise<geometry_msgs::Twist>((publishedName + "/skidsteer"), 10);
    infoLogPublisher = sNH.advertise<std_msgs::String>("/infoLog", 1, true);

    float heartbeat_publish_interval = 2;
    publish_heartbeat_timer = sNH.createTimer(ros::Duration(heartbeat_publish_interval), &sbridge::publishHeartBeatTimerEventHandler, this);


     ROS_INFO("constructor");

}

void sbridge::cmdHandler(const geometry_msgs::Twist::ConstPtr& message) {
    double linearVel = (message->linear.x);
    double angularVel = (message->angular.z);


    float turn = angularVel/60;
    float forward = linearVel/355;

    if (forward >= 150){

        forward -= (abs(turn)/5);
    }

    if (linearVel >= 0 && forward <= 0)
    {
        forward = 0;
    }
    if (linearVel <= 0 && forward >= 0)
    {
        forward = 0;
    }


    velocity.linear.x = forward,
            velocity.angular.z = turn;
    skidsteerPublish.publish(velocity);
}

void sbridge::publishHeartBeatTimerEventHandler(const ros::TimerEvent& event) {
    std_msgs::String msg;
    msg.data = "";
    heartbeatPublisher.publish(msg);

    ROS_INFO("%ds, %dnsec", event.last_real.sec, event.last_real.nsec);
}

sbridge::~sbridge() {
}
