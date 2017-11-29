//This file handles communication between the gazebo simulation and the rover

#include "sbridge.h"

//sbridge constructor
sbridge::sbridge(std::string publishedName) {

    //Create a new node that will communicate with ROS
    ros::NodeHandle sNH;

    //Create a drive control subscriber
    //publishedName + '/driveControl" == the topic to subscribe to
    //10 == the incoming message queue size
    //&sbridge::cmdHandler ==  callback to class method first parameter
    //this == calllback to class method second parameter
    driveControlSubscriber = sNH.subscribe((publishedName + "/driveControl"), 10, &sbridge::cmdHandler, this);

    //Create a heatbeat publisher to show that the node is still alive and running
    //publishedName + "/sbridge/heartbeat" == topic to publish to
    //1 == the outgoing message queue
    //false == disables the latching connection. It does not save the last message nor automatically send
    //automatically send it to future connecting subscribers
    //this parameter is optional
    heartbeatPublisher = sNH.advertise<std_msgs::String>((publishedName + "/sbridge/heartbeat"), 1, false);

    //Create a skidsteer publisher to
    skidsteerPublish = sNH.advertise<geometry_msgs::Twist>((publishedName + "/skidsteer"), 10);

    //Create a infoLog publisher to send log information to the log box in the GUI
    infoLogPublisher = sNH.advertise<std_msgs::String>("/infoLog", 1, true);

    //Publishes a heartbeat every 2 seconds
    float heartbeat_publish_interval = 2;

    //Schedules a callback at the heartbeat_publish_interval
    //This is not a realtime timer
    //ros::Duration(heartbeat_publish_interval) == period between calls to the callback
    //&sbridge::publishHeartBeatTimerEventHandler == callback to class method first parameter
    //this == callback to class method second parameter
    publish_heartbeat_timer = sNH.createTimer(ros::Duration(heartbeat_publish_interval), &sbridge::publishHeartBeatTimerEventHandler, this);

    //Log information
    ROS_INFO("constructor");

}

//Command Handler
void sbridge::cmdHandler(const geometry_msgs::Twist::ConstPtr& message) {
    double left = (message->linear.x);
    double right = (message->angular.z);
    
    float max_turn_rate = 4.5; //radians per second
    float max_linear_velocity = 0.65; // meters per second

    float turn = 0;
    float forward = 0;

    float linearVel = (left + right)/2;
    float angularVel = (right-left)/2;

    turn = angularVel/55;
    forward = linearVel/390;
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

    //If the absolute value of the forward value (float) is greater than or equal to the max_linear_velocity
    if (fabs(forward) >= max_linear_velocity) {
        //Set the forward value to
        forward = forward/fabs(forward) * max_linear_velocity;
    }

    if (fabs(turn) >= max_turn_rate) { //max value needs tuning
        turn = turn/fabs(turn) * max_turn_rate;
    }

    velocity.linear.x = forward,
            velocity.angular.z = turn;
    skidsteerPublish.publish(velocity);
}

//
void sbridge::publishHeartBeatTimerEventHandler(const ros::TimerEvent& event) {
    std_msgs::String msg;
    msg.data = "";
    //Publish the heartbeat message
    heartbeatPublisher.publish(msg);
    //Log information for the timer in seconds and nanoseconds
    ROS_INFO("%ds, %dnsec", event.last_real.sec, event.last_real.nsec);
}

//Destructor
sbridge::~sbridge() {
}
