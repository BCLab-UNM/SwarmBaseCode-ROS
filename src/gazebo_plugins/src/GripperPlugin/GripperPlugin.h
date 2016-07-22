#ifndef GRIPPER_PLUGIN_H
#define GRIPPER_PLUGIN_H

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/msgs/msgs.hh>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <std_msgs/Float32.h>
#include <thread>
#include <iostream>
#include <string>
#include "GripperManager.h"

namespace gazebo {

class GripperPlugin : public ModelPlugin {

  public:

    // required overloaded function from ModelPlugin class
    void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf);

    // Gazebo actuation function
    void updateWorldEventHandler();

    // ROS topic handlers
    void setWristAngleHandler(const std_msgs::Float32ConstPtr &msg);
    void setFingerAngleHandler(const std_msgs::Float32ConstPtr &msg);

  private:

    void processRosQueue();
    gazebo::physics::JointPtr loadJoint(std::string jointTag);

    physics::ModelPtr model;
    sdf::ElementPtr sdf;

    // interface for processing ROS message queue
    event::ConnectionPtr updateConnection;
    std::unique_ptr<ros::NodeHandle> rosNode;
    std::thread rosQueueThread;
    ros::CallbackQueue rosQueue;

    // ROS subscribers
    ros::Subscriber wristAngleSubscriber;
    ros::Subscriber fingerAngleSubscriber;

    // gripper component objects
    GripperManager gripperManager;
    gazebo::physics::JointPtr wristJoint;
    gazebo::physics::JointPtr leftFingerJoint;
    gazebo::physics::JointPtr rightFingerJoint;

    math::Angle desiredWristAngle;
    math::Angle desiredFingerAngle;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GripperPlugin)
  }

#endif /* GRIPPER_PLUGIN_H */
