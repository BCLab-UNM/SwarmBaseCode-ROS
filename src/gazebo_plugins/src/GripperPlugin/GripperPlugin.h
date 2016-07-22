#ifndef GRIPPER_PLUGIN_H
#define GRIPPER_PLUGIN_H

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/Float32.h>
#include <thread>
#include "GripperManager.h"

/**
 * This class implements a gripper plugin for the NASA Swarmathon Rovers.
 *
 * <p>The gripper design consists of a wrist which rotates the gripper up or
 * down about a fixed axis. The gripper itself is composed of two fingers which
 * open and close symmetrically.
 *
 * <p>The gripper is designed to use a base PID controller for each of the
 * three primary joints. A gripper manager is implemented to pass force and
 * angle data between this class and the PID controllers.
 *
 * @author Matthew Fricke
 * @author Antonio Griego
 * @see    ModelPlugin
 * @see    GripperManager
 * @see    PIDController
 */
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

        // private helper functions
        void processRosQueue();
        physics::JointPtr loadJoint(std::string jointTag);

        // pointers to gazebo model and xml configuration file
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
        physics::JointPtr wristJoint;
        physics::JointPtr leftFingerJoint;
        physics::JointPtr rightFingerJoint;
        math::Angle desiredWristAngle;
        math::Angle desiredFingerAngle;

        common::Time previousUpdateTime;
        float updateRateInSeconds;
        bool isDebuggingModeActive;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(GripperPlugin)
}

#endif /* GRIPPER_PLUGIN_H */
