#ifndef GRIPPER_PLUGIN_H
#define GRIPPER_PLUGIN_H

#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/Float32.h>
#include <thread>
#include "GripperManager.h"
#include <string>
#include <mutex>

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
 * <p>In order to maintain a stable contact with the target a joint
 * is created between the gripper and the target. This is necessary
 * because Gazebo 2.2 was not designed for gripper physics.
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

    // For sending informational messages to the UI
    void sendInfoLogMessage(std::string text);
    
      // required overloaded function from ModelPlugin class
      void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf);

      // Gazebo actuation function
      void updateWorldEventHandler();
      void updateGraspedStaticTargetPose();
      
      // ROS topic handlers
      void setWristAngleHandler(const std_msgs::Float32ConstPtr &msg);
      void setFingerAngleHandler(const std_msgs::Float32ConstPtr &msg);

      void rightFingerContactEventHandler(ConstContactsPtr& msg);
      void leftFingerContactEventHandler(ConstContactsPtr& msg);

      ~GripperPlugin();

    private:

      // private helper functions
      void processRosQueue();
      void loadDebugMode();
      void loadUpdatePeriod();
      std::string loadSubscriptionTopic(std::string topicTag);
      physics::JointPtr loadJoint(std::string jointTag);
      PIDController::PIDSettings loadPIDSettings(std::string PIDTag);
      void handleGrasping();

      void attach();
      void detach();

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

      // ROS Publishers
      ros::Publisher infoLogPublisher;

      // gripper component objects
      GripperManager gripperManager;
      physics::JointPtr wristJoint;
      physics::JointPtr leftFingerJoint;
      physics::JointPtr rightFingerJoint;
      math::Angle desiredWristAngle;
      math::Angle desiredFingerAngle;

      // gripper grasping objects
      physics::Model_V modelList;

      // time management variables
      common::Time previousUpdateTime;
      float updatePeriodInSeconds;

      // debugging variables
      common::Time previousDebugUpdateTime;
      float debugUpdatePeriodInSeconds;
      bool isDebuggingModeActive;


      // *************************************
      // Gripper - Target Attachment Variables
      // *************************************

      // Subscribe to the contact sensors defined in the model file
      gazebo::transport::SubscriberPtr rightFingerContactsSubscriber;
      gazebo::transport::SubscriberPtr leftFingerContactsSubscriber;

      // How long to be in contact before attaching the gripper to the target
      common::Time contactThreshold;

      // How long to not be in contact before detaching the gripper from the target
      common::Time noContactThreshold;

      // Time spent with both fingers in contact with the same target
      common::Time contactTime;

      // Time spent with either finger not in contact with the same target
      // as the other finger
      common::Time noContactTime;

      common::Time prevHandleGraspingTime;

      // indicator of whether there is an active joint attachment between
      // the gripper and some target
      bool isAttached;

      // Link pointers for gripper attachment
      // There is a left and right link for the left and right fingers
      physics::LinkPtr gripperAttachLink;

      // The model object for the grasped target
      physics::ModelPtr attachedTargetModel;

      // A pose offset so we can move grasped static objects around
      math::Pose attachedTargetOffset;
      
      // These pointers are only when a finger is in contact with a target
      // object
      physics::LinkPtr rightFingerTargetLink;
      physics::LinkPtr leftFingerTargetLink;
     
      // Timers used to decide when a finger is in contact with an object
      // These are used to smooth out the noisiness of the gazebo engine
      common::Time fingerNoContactThreshold;
      common::Time leftFingerNoContactTime;
      common::Time rightFingerNoContactTime;

      // Target attach joint
      physics::JointPtr targetAttachJoint;

      // Make sure the attach link doesn't change while attaching to it
      std::mutex attaching_mutex;

      bool dropStaticTarget;
      gazebo::math::Angle maxGrippingAngle;

      int  dropStaticTargetCounter;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(GripperPlugin)

}

#endif /* GRIPPER_PLUGIN_H */
