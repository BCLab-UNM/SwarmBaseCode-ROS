// This file handles communication between the gazebo simulation and the rover

// Since the simulation cannot get data from the real rovers, this package converts
// the behavior package data into readable data for the gazebo simulation to understand
#include "sbridge.h"

// sbridge constructor
sbridge::sbridge(std::string publishedName) {

    // Create a new node that will communicate with ROS
    ros::NodeHandle sNH;


    // Create a drive control subscriber
    // publishedName + '/driveControl" == the topic to subscribe to
    // 10 == the incoming message queue size
    // &sbridge::cmdHandler ==  callback to class method first parameter
    // this == calllback to class method second parameter
    driveControlSubscriber = sNH.subscribe((publishedName + "/driveControl"), 10, &sbridge::cmdHandler, this);

    // Create a heatbeat publisher to show that the node is still alive and running
    // publishedName + "/sbridge/heartbeat" == topic to publish to
    // 1 == the outgoing message queue
    // false == disables the latching connection. It does not save the last message nor automatically send
    // automatically send it to future connecting subscribers
    // this parameter is optional
    heartbeatPublisher = sNH.advertise<std_msgs::String>((publishedName + "/sbridge/heartbeat"), 1, false);

    // Create a skidsteer publisher to
    skidsteerPublish = sNH.advertise<geometry_msgs::Twist>((publishedName + "/skidsteer"), 10);

    // Create a infoLog publisher to send log information to the log box in the GUI
    infoLogPublisher = sNH.advertise<std_msgs::String>("/infoLog", 1, true);

    // Publishes a heartbeat every 2 seconds
    float heartbeat_publish_interval = 2;

    // Schedules a callback at the heartbeat_publish_interval
    // This is not a realtime timer
    // ros::Duration(heartbeat_publish_interval) == period between calls to the callback
    // &sbridge::publishHeartBeatTimerEventHandler == callback to class method first parameter
    // this == callback to class method second parameter
    publish_heartbeat_timer = sNH.createTimer(ros::Duration(heartbeat_publish_interval), &sbridge::publishHeartBeatTimerEventHandler, this);

    ROS_INFO("constructor");

}

// Command Handler
// PWM (pulse with modulation) values to linear values
void sbridge::cmdHandler(const geometry_msgs::Twist::ConstPtr& message) {
    // Gets and sets the linear PWM value
    double left = (message->linear.x);
    // Gets and sets the angular PWM value
    double right = (message->angular.z);
    
    // Set max values
    float max_turn_rate = 4.5; // radians per second
    float max_linear_velocity = 0.65; //  meters per second

    // Temperary Variables to store the velocity values
    // Angular
    float turn = 0;
    // Linear
    float forward = 0;

    float linearVel = (left + right)/2;
    float angularVel = (right-left)/2;

    // Converts the PWM values to velocity values
    // Necessary hand-tuned conversion value
    turn = angularVel/55;
    // Necessary hand-tuned conversion value
    forward = linearVel/390;

    // If the rover is moving forward and turning at the same time, it will not have enough power to carry out
    // each of those at it's full values, so we need to lower this value so the PIDs can compensate for both
    // the linear and angular velocity
    if (forward >= 150){
      // Lower forward velocity value by necessary hand-tuned conversion value
      forward -= (abs(turn)/5);
    }

    // If the linear velocity (PWM value) is moving in the forward direction (value is greater than or equal to 0),
    // but forward (velocity) gets converted to a non-positive value (for reverse movement)
    if (linearVel >= 0 && forward <= 0)
    {
      // Do not move (set forward velocity to 0)
      forward = 0;
    }

    // If the linear velocity (PWM value) is moving in the reverse direction (value is greater than or equal to 0),
    // but the foward (velocity) variable gets converted to a non-negative value (for forwards movement)
    if (linearVel <= 0 && forward >= 0)
    {
      // Do not move (set forward velocity to 0)
      forward = 0;
    }

    // If the absolute value of the forward value (float) is greater than or equal to the max_linear_velocity
    if (fabs(forward) >= max_linear_velocity) {
        forward = forward/fabs(forward) * max_linear_velocity;
    }

    if (fabs(turn) >= max_turn_rate) { // max value needs tuning
        turn = turn/fabs(turn) * max_turn_rate;
    }

    // Sets the converted (PWM to velocity) values into the variable that the simulation can read
    // Linear velocity
    velocity.linear.x = forward,
            // Angular velocity
            velocity.angular.z = turn;
    // Publish velocity information to rotary wheel movement
    skidsteerPublish.publish(velocity);
}


void sbridge::publishHeartBeatTimerEventHandler(const ros::TimerEvent& event) {
    std_msgs::String msg;
    msg.data = "";
    // Publish the heartbeat message
    heartbeatPublisher.publish(msg);

    // Log information for the timer in seconds and nanoseconds
    ROS_INFO("%ds, %dnsec", event.last_real.sec, event.last_real.nsec);
}

// Destructor
sbridge::~sbridge() {
}
