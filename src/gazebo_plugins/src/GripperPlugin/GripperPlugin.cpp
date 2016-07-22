#include "GripperPlugin.h"

using namespace gazebo;
using namespace std;

void GripperPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  model = _model;
  sdf = _sdf;
  isDebuggingModeActive = false;
  previousUpdateTime = model->GetWorld()->GetSimTime();

  // print debug statements if toggled in model xml configuration file
  if(sdf->HasElement("debug")) {
    string debugString = sdf->GetElement("debug")->Get<string>();
    if(debugString.compare("true") == 0) {
      isDebuggingModeActive = true;
      if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
      }
    }
  } else {
    ROS_INFO_STREAM("[Gripper Plugin : " << model->GetName() << "]: missing <debug> tag, defaulting to \"false\"");
  }

  ROS_DEBUG_STREAM_COND(isDebuggingModeActive, "[Gripper Plugin : " << model->GetName() << "]: begin loading gripper plugin");

  if(!sdf->HasElement("updateRateInSeconds")) {
    ROS_INFO_STREAM("[Gripper Plugin : " << model->GetName() << "]: missing <updateRateInSeconds> tag, defaulting to 1 second");
    updateRateInSeconds = 1.0;
  } else {
    updateRateInSeconds = sdf->GetElement("updateRateInSeconds")->Get<float>();
  }

  wristJoint = loadJoint("wristJoint");
  leftFingerJoint = loadJoint("leftFingerJoint");
  rightFingerJoint = loadJoint("rightFingerJoint");

  updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&GripperPlugin::updateWorldEventHandler, this));

  if (!ros::isInitialized()) {
    ROS_DEBUG_STREAM_COND(isDebuggingModeActive, "[Gripper Plugin]: initializing ROS");
    int argc = 0;
    char **argv=NULL;
    ros::init(argc, argv, model->GetName() + "_gripper", ros::init_options::NoSigintHandler);
  }

  rosNode.reset(new ros::NodeHandle(string(model->GetName()) + "_gripper"));

  string wristTopic, fingerTopic;

  if(sdf->HasElement("wristTopic")) {
    wristTopic = sdf->GetElement("wristTopic")->Get<std::string>();
  } else {
    ROS_ERROR_STREAM("[Gripper Plugin]: In GripperPlugin.cpp: Load(): No <wristTopic> tag defined in model SDF file");
    exit(1);
  }

  if(sdf->HasElement("fingerTopic")) {
    fingerTopic = sdf->GetElement("fingerTopic")->Get<std::string>();
  } else {
    ROS_ERROR_STREAM("[Gripper Plugin]: In GripperPlugin.cpp: Load(): No <fingerTopic> tag defined in model SDF file");
    exit(1);
  }

  // subscribe to ROS topics - begin
  ros::SubscribeOptions wristSubscriptionOptions = ros::SubscribeOptions::create<std_msgs::Float32>(wristTopic, 1, boost::bind(&GripperPlugin::setWristAngleHandler, this, _1), ros::VoidPtr(), &rosQueue);
  ros::SubscribeOptions fingerSubscriptionOptions = ros::SubscribeOptions::create<std_msgs::Float32>(fingerTopic, 1, boost::bind(&GripperPlugin::setFingerAngleHandler, this, _1), ros::VoidPtr(), &rosQueue);

  wristAngleSubscriber = rosNode->subscribe(wristSubscriptionOptions);
  fingerAngleSubscriber = rosNode->subscribe(fingerSubscriptionOptions);
  // subscribe to ROS topics - end

  // spin up the queue helper thread
  rosQueueThread = std::thread(std::bind(&GripperPlugin::processRosQueue, this));

  ROS_DEBUG_STREAM_COND(isDebuggingModeActive, "[Gripper Plugin : " << model->GetName() << "]: finished loading gripper plugin");
}

void GripperPlugin::updateWorldEventHandler() {
  common::Time currentTime = model->GetWorld()->GetSimTime();

  if((currentTime - previousUpdateTime).Float() < updateRateInSeconds) {
    return;
  }

  previousUpdateTime = currentTime;

  GripperState currentState;
  GripperState desiredState;
    
  // get the current gripper state
  currentState.wristAngle = wristJoint->GetAngle(0).Radian();
  currentState.leftFingerAngle = leftFingerJoint->GetAngle(0).Radian();
  currentState.rightFingerAngle = rightFingerJoint->GetAngle(0).Radian();

  // get the total angle of both fingers,
  // the right finger joint angle is always negative
  // the left finger joint angle is always positive
  // the total finger angle = left finger joint angle - (-right finger joint angle)
  // the total finger angle is ALWAYS POSITIVE (or zero)
  float leftFingerAngle = leftFingerJoint->GetAngle(0).Radian();
  float rightFingerAngle = rightFingerJoint->GetAngle(0).Radian();

  // Set the desired gripper state
  desiredState.leftFingerAngle = desiredFingerAngle.Radian() / 2.0;
  desiredState.rightFingerAngle = -desiredFingerAngle.Radian() / 2.0;
  desiredState.wristAngle = desiredWristAngle.Radian();
  
  // Get the forces to apply to the joints from the PID controllers
  GripperForces commandForces = gripperManager.getForces(desiredState, currentState);

  // If debugging mode is active, print debugging statements
  ROS_DEBUG_STREAM_COND(isDebuggingModeActive, "[Gripper Plugin : " << model->GetName() << "]");
  ROS_DEBUG_STREAM_COND(isDebuggingModeActive, "   Wrist Angle: Current Angle: " << setw(12) << currentState.wristAngle        << " rad");
  ROS_DEBUG_STREAM_COND(isDebuggingModeActive, "                Desired Angle: " << setw(12) << desiredState.wristAngle        << " rad");
  ROS_DEBUG_STREAM_COND(isDebuggingModeActive, "                Applied Force: " << setw(12) << commandForces.wristForce       << " N");
  ROS_DEBUG_STREAM_COND(isDebuggingModeActive, "L Finger Angle: Current Angle: " << setw(12) << currentState.leftFingerAngle   << " rad");
  ROS_DEBUG_STREAM_COND(isDebuggingModeActive, "                Desired Angle: " << setw(12) << desiredState.leftFingerAngle   << " rad");
  ROS_DEBUG_STREAM_COND(isDebuggingModeActive, "                Applied Force: " << setw(12) << commandForces.leftFingerForce  << " N");
  ROS_DEBUG_STREAM_COND(isDebuggingModeActive, "R Finger Angle: Current Angle: " << setw(12) << currentState.rightFingerAngle  << " rad");
  ROS_DEBUG_STREAM_COND(isDebuggingModeActive, "                Desired Angle: " << setw(12) << desiredState.rightFingerAngle  << " rad");
  ROS_DEBUG_STREAM_COND(isDebuggingModeActive, "                Applied Force: " << setw(12) << commandForces.rightFingerForce << " N\n");

  // Apply the command forces to the joints
  wristJoint->SetForce(0, commandForces.wristForce);
  leftFingerJoint->SetForce(0, commandForces.leftFingerForce);
  rightFingerJoint->SetForce(0, commandForces.rightFingerForce);
}

void GripperPlugin::setWristAngleHandler(const std_msgs::Float32ConstPtr& msg) {
  float wristAngle = msg->data;
  desiredWristAngle = wristAngle;
}

void GripperPlugin::setFingerAngleHandler(const std_msgs::Float32ConstPtr& msg) {
  float fingerAngle = msg->data;
  desiredFingerAngle = fingerAngle;
}

/**
 * This function loads a joint and joint name for the specified elements.
 * During the current development phase of this plugin, message output is
 * especially verbose to facilitate debugging.
 */
physics::JointPtr GripperPlugin::loadJoint(std::string jointTag) {
  string jointName;
  physics::JointPtr joint;
	    
  if(sdf->HasElement(jointTag)) {
    jointName = sdf->GetElement(jointTag)->Get<std::string>();
    joint = this->model->GetJoint(jointName);
		
    if(!joint) {
      ROS_ERROR_STREAM("[Gripper Plugin : " << model->GetName() << "]: In GripperPlugin.cpp: loadJoint(): No " << jointName << " joint defined in model SDF file");
      exit(1);
    }
  } else {
      ROS_ERROR_STREAM("[Gripper Plugin : " << model->GetName() << "]: In GripperPlugin.cpp: loadJoint(): No <" << jointTag << "> tag defined in model SDF file");
      exit(1);
  }

  return joint;
}

void GripperPlugin::processRosQueue() {
  static const double timeout = .01;
  while (rosNode->ok()) {
    rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}
