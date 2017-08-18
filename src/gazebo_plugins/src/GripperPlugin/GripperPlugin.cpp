#include <std_msgs/String.h>
#include <math.h> // For Vector3
#include "GripperPlugin.h"
#include <sstream>

using namespace gazebo;
using namespace std;

/**
 * This class is inherited from the ModelPlugin class and implemented here.
 * It loads all necessary data for the plugin from the provided model and SDF
 * parameters. In addition, this function sets up the two required subscribers
 * needed for ROS to communicate with the gripper for a rover.
 *
 * <p> In the event that a required XML tag is not found in the SDF file the
 * plugin will initiate an exit(1) call with extreme prejudice resulting,
 * typically, in a segmentation fault.
 *
 * This class has two main jobs. It passes gripper commands from ROS to the
 * GripperManager which returns actution commands for gazebo.
 * And this class attached and detaches targets to the gripper as needed.
 * This is required because older versions of Gazebo do not implement gripper
 * physics.
 *
 * @param _model The Swarmie Rover model this gripper is attached to.
 * @param _sdf   The SDF configuration file for this plugin the model.
 */
void GripperPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  model = _model;
  sdf = _sdf;
  previousUpdateTime = model->GetWorld()->GetSimTime();
  previousDebugUpdateTime = model->GetWorld()->GetSimTime();

  attachedTargetModel = NULL;
  dropStaticTarget = false;
  dropStaticTargetCounter = 0;
  
  // 1.39626 is approximately equal to 80 degrees
  maxGrippingAngle = 1.39626;

  // Set values for the gripper attachment code
  isAttached = false;
  noContactTime = common::Time(0.0);
  contactTime = common::Time(0.0);
  contactThreshold = common::Time(0.0001);
  noContactThreshold = common::Time(0.1);
  fingerNoContactThreshold = common::Time(0.1);
  prevHandleGraspingTime = model->GetWorld()->GetSimTime();
    
  // Create a ros node
  rosNode.reset(new ros::NodeHandle(string(model->GetName()) + "_gripper"));
  ROS_DEBUG_STREAM_COND(isDebuggingModeActive, "[Gripper Plugin : "
    << model->GetName() << "]\n    initialize a NodeHandle for this plugin:\n"
    << "        " << model->GetName() + "_gripper");

  // Create publisher so we can send info messages to the UI
  infoLogPublisher = rosNode->advertise<std_msgs::String>("/infoLog", 1, true);
  // print debug statements if toggled to "true" in the model SDF file
  loadDebugMode();

  ROS_DEBUG_STREAM_COND(isDebuggingModeActive, "[Gripper Plugin : "
    << model->GetName() << "]\n    ===== BEGIN LOADING =====");

  // set the update period (number of updates per second) for this plugin
  loadUpdatePeriod();
  ROS_DEBUG_STREAM_COND(isDebuggingModeActive, "[Gripper Plugin : "
    << model->GetName() << "]\n    set the plugin update period:\n"
    << "        " << updatePeriodInSeconds << " Hz or " << (1.0/updatePeriodInSeconds)
    << " updates per second");

  // LOAD GRIPPER JOINTS - begin
  wristJoint = loadJoint("wristJoint");
  leftFingerJoint = loadJoint("leftFingerJoint");
  rightFingerJoint = loadJoint("rightFingerJoint");
  ROS_DEBUG_STREAM_COND(isDebuggingModeActive, "[Gripper Plugin : "
    << model->GetName() << "]\n    loaded the gripper's joints:\n"
    << "        " << wristJoint->GetName() << endl << "        "
    << leftFingerJoint->GetName() << endl << "        "
    << rightFingerJoint->GetName());
  // LOAD GRIPPER JOINTS - end

  // Load gripper links - begin
  // Just need the target attachment link
  gripperAttachLink = model->GetChildLink("gripper_wrist");
  // Load gripper links - end
  
  // INITIALIZE GRIPPER MANAGER - begin
  PIDController::PIDSettings wristPID = loadPIDSettings("wrist");
  PIDController::PIDSettings fingerPID = loadPIDSettings("finger");
  gripperManager = GripperManager(wristPID, fingerPID);
  ROS_DEBUG_STREAM_COND(isDebuggingModeActive, "[Gripper Plugin : "
    << model->GetName() << "]\n    initialized the GripperManager:\n"
    << "        wristPID:  Kp=" << wristPID.Kp << ", Ki=" << wristPID.Ki
    << ", Kd=" << wristPID.Kd << ", force min=" << wristPID.min
    << ", force max=" << wristPID.max << ", dt=" << wristPID.dt << endl
    << "        fingerPID: Kp=" << fingerPID.Kp << ", Ki=" << fingerPID.Ki
    << ", Kd=" << fingerPID.Kd << ", force min=" << fingerPID.min
    << ", force max=" << fingerPID.max << ", dt=" << fingerPID.dt);
  // INITIALIZE GRIPPER MANAGER - end

  // Connect the updateWorldEventHandler function to Gazebo;
  // ConnectWorldUpdateBegin sets our handler to be called at the beginning of
  // each physics update iteration
  updateConnection = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&GripperPlugin::updateWorldEventHandler, this)
  );
  ROS_DEBUG_STREAM_COND(isDebuggingModeActive, "[Gripper Plugin : "
    << model->GetName() << "]\n    bind world update function to gazebo:\n"
    << "        void GripperPlugin::updateWorldEventHandler()");

  // ROS must be initialized in order to set up this plugin's subscribers
  if (!ros::isInitialized()) {
    ROS_ERROR_STREAM("[Gripper Plugin : " << model->GetName()
      << "] In GripperPlugin.cpp: Load(): ROS must be initialized before "
      << "this plugin can be used!");
    exit(1);
  }

  // SUBSCRIBE TO ROS TOPICS - begin
  string wristTopic = loadSubscriptionTopic("wristTopic");
  ros::SubscribeOptions wristSubscriptionOptions =
    ros::SubscribeOptions::create<std_msgs::Float32>(
      wristTopic, 1,
      boost::bind(&GripperPlugin::setWristAngleHandler, this, _1),
      ros::VoidPtr(), &rosQueue
    );

  string fingerTopic = loadSubscriptionTopic("fingerTopic");
  ros::SubscribeOptions fingerSubscriptionOptions =
    ros::SubscribeOptions::create<std_msgs::Float32>(
      fingerTopic, 1,
      boost::bind(&GripperPlugin::setFingerAngleHandler, this, _1),
      ros::VoidPtr(), &rosQueue
    );

  wristAngleSubscriber = rosNode->subscribe(wristSubscriptionOptions);
  fingerAngleSubscriber = rosNode->subscribe(fingerSubscriptionOptions);

  ROS_DEBUG_STREAM_COND(isDebuggingModeActive, "[Gripper Plugin : "
    << model->GetName() << "]\n    subscribe to all gripper topics:\n"
    << "        " << wristTopic << endl << "        " << fingerTopic);
  // SUBSCRIBE TO ROS TOPICS - end

  // spin up the queue helper thread
  rosQueueThread =
    std::thread(std::bind(&GripperPlugin::processRosQueue, this));
  ROS_DEBUG_STREAM_COND(isDebuggingModeActive, "[Gripper Plugin : "
    << model->GetName() << "]\n    bind queue helper function to private "
    << "thread:\n        void GripperPlugin::processRosQueue()");

  // Create Gazebo node and init
// Create Gazebo node and init
  gazebo::transport::NodePtr gazeboNode(new gazebo::transport::Node());
  gazeboNode->Init();
  
  string leftFingerContactTopic = "/gazebo/default/"+model->GetName()+"/gripper_left_finger/finger/contacts";
  string rightFingerContactTopic = "/gazebo/default/"+model->GetName()+"/gripper_right_finger/finger/contacts";

  
  // Subscribe to the gripper contact gazebo topics
  rightFingerContactsSubscriber = gazeboNode->Subscribe(rightFingerContactTopic, &GripperPlugin::rightFingerContactEventHandler, this);
  sendInfoLogMessage("GripperPlugin subscribed to " + rightFingerContactTopic);
  
  leftFingerContactsSubscriber = gazeboNode->Subscribe(leftFingerContactTopic, &GripperPlugin::leftFingerContactEventHandler, this);
  sendInfoLogMessage("GripperPlugin subscribed to " + leftFingerContactTopic);
  
  ROS_DEBUG_STREAM_COND(isDebuggingModeActive, "[Gripper Plugin : "
    << model->GetName() << "]\n    ===== FINISHED LOADING =====");
}

/**
 * This function handles updates to the gripper plugin. It is called by the
 * Gazebo physics engine at the start of each update iteration. The subscribers
 * will handle updating the desiredWristAngle and desiredFingerAngle variables.
 * This function will take those updated values and apply them to the joints
 * of the gripper as needed to instigate the desired movements requested by the
 * gripper publishers.
 */
void GripperPlugin::updateWorldEventHandler() {
  common::Time currentTime = model->GetWorld()->GetSimTime();

  // only update the gripper plugin once every updatePeriodInSeconds
  if((currentTime - previousUpdateTime).Float() < updatePeriodInSeconds) {
    return;
  }

  // grasp an object if conditions are met
  handleGrasping();

  // Moves static models when grasped. Does nothing if the model is non-static
  updateGraspedStaticTargetPose();

  previousUpdateTime = currentTime;

  GripperManager::GripperState currentState;
  GripperManager::GripperState desiredState;

  // get the current gripper state
  currentState.wristAngle = wristJoint->GetAngle(0).Radian();
  currentState.leftFingerAngle = leftFingerJoint->GetAngle(0).Radian();
  currentState.rightFingerAngle = rightFingerJoint->GetAngle(0).Radian();

  // get the total angle of both fingers:
  // => right finger joint angle is always negative
  // => left finger joint angle is always positive
  // total finger angle = left finger joint angle - (-right finger joint angle)
  // total finger angle is ALWAYS POSITIVE (or zero)
  float leftFingerAngle = leftFingerJoint->GetAngle(0).Radian();
  float rightFingerAngle = rightFingerJoint->GetAngle(0).Radian();

  // Set the desired gripper state
  desiredState.leftFingerAngle = desiredFingerAngle.Radian() / 2.0;
  desiredState.rightFingerAngle = -desiredFingerAngle.Radian() / 2.0;
  desiredState.wristAngle = desiredWristAngle.Radian();
  
  // Get the forces to apply to the joints from the PID controllers
  GripperManager::GripperForces commandForces =
    gripperManager.getForces(desiredState, currentState);

  // Apply the command forces to the joints
  wristJoint->SetForce(0, commandForces.wristForce);
  leftFingerJoint->SetForce(0, commandForces.leftFingerForce);
  rightFingerJoint->SetForce(0, commandForces.rightFingerForce);

  // If debugging mode is active, print debugging statements
  if((currentTime - previousDebugUpdateTime).Float() >= debugUpdatePeriodInSeconds) {
    previousDebugUpdateTime = currentTime;

    ROS_DEBUG_STREAM_COND(
      isDebuggingModeActive, "[Gripper Plugin : "
      << model->GetName() << "]\n"
      << "           Wrist Angle: Current Angle: " << setw(12)
      << currentState.wristAngle        << " rad\n"
      << "                        Desired Angle: " << setw(12)
      << desiredState.wristAngle        << " rad\n"
      << "                        Applied Force: " << setw(12)
      << commandForces.wristForce       << " N\n"
      << "     Left Finger Angle: Current Angle: " << setw(12)
      << currentState.leftFingerAngle   << " rad\n"
      << "                        Desired Angle: " << setw(12)
      << desiredState.leftFingerAngle   << " rad\n"
      << "                        Applied Force: " << setw(12)
      << commandForces.leftFingerForce  << " N\n"
      << "    Right Finger Angle: Current Angle: " << setw(12)
      << currentState.rightFingerAngle  << " rad\n"
      << "                        Desired Angle: " << setw(12)
      << desiredState.rightFingerAngle  << " rad\n"
      << "                        Applied Force: " << setw(12)
      << commandForces.rightFingerForce << " N\n"
    );
  }
}

/**
 * This is the subscriber function for the desiredWristAngle variable. Updates
 * to desiredWristAngle will cause the gripper to be moved vertically around
 * its axis of rotation.
 *
 * @param msg A publisher message consisting of a postive floating point value
 *            which represents an angle in radians.
 */
void GripperPlugin::setWristAngleHandler(const std_msgs::Float32ConstPtr& msg) {
  float wristAngle = msg->data;
  desiredWristAngle = wristAngle;
}

/**
 * This is the subscriber function for the desiredFingerAngle variable. Updates
 * to desiredFingerAngle will cause the gripper to open or close its fingers
 * based on the change in angle.
 *
 * @param msg A publisher message consisting of a postive floating point value
 *            which represents an angle in radians.
 */ 
void GripperPlugin::setFingerAngleHandler(const std_msgs::Float32ConstPtr& msg) {

  double fingerAngle = msg->data;
  desiredFingerAngle = fingerAngle;

  // Force drop static models when the gripper is open
  if (isAttached) {

    if (attachedTargetModel->IsStatic() && desiredFingerAngle >= maxGrippingAngle) {
      sendInfoLogMessage("GripperPlugin: detach() trying to detach due to finger angle failsafe");
      try {
        detach(); 
      } catch (exception &e) {
	  sendInfoLogMessage("GripperPlugin: detach() failed with: " + string(e.what()));
      }
    }
  }
}

/**
 * This function is used inside of a custom thread to process the messages
 * being passed from the publishers to the subscribers. It is the interface
 * between ROS and the setWristAngleHandler() and setFingerAngleHandler()
 * functions.
 */
void GripperPlugin::processRosQueue() {
  static const double timeout = 0.01;
  while (rosNode->ok()) {
    rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}

/**
 * This function sets the "isDebuggingModeActive" flag to true or false
 * depending on the <debug> tag for this plugin in the configuration SDF file.
 * By default, the value of "isDebuggingModeActive" is false.
 *
 * <p>When debugging mode is active, extra print statements with the current
 * status of the gripper's joints will be printed to the console. These
 * print statements will occur at a rate of once per 3 seconds (simulated time)
 * or as defined by the user.
 *
 * <p>For example:
 * <p><debug>
 * <p>    <printToConsole>true</printToConsole>
 * <p>    <printDelayInSeconds>5.0</printDelayInSeconds>
 * <p></debug>
 */
void GripperPlugin::loadDebugMode() {
  isDebuggingModeActive = false;

  if(sdf->HasElement("debug")) {
    sdf::ElementPtr debug = sdf->GetElement("debug");

    if(debug->HasElement("printToConsole")) {
      string debugString = debug->GetElement("printToConsole")->Get<string>();

      if(debugString.compare("true") == 0) {
        isDebuggingModeActive = true;

        if(debug->HasElement("printDelayInSeconds")) {
          debugUpdatePeriodInSeconds =
            debug->GetElement("printDelayInSeconds")->Get<float>();

          // fatal error: the debugUpdatePeriodInSeconds cannot be <= 0
          if(debugUpdatePeriodInSeconds <= 0.0) {
            ROS_ERROR_STREAM("[Gripper Plugin : " << model->GetName()
              << "]: In GripperPlugin.cpp: loadDebugMode(): "
              << "printDelayInSeconds = " << debugUpdatePeriodInSeconds
              << ", printDelayInSeconds cannot be <= 0.0");
            exit(1);
          }
        } else {
          ROS_INFO_STREAM("[Gripper Plugin : " << model->GetName()
            << "]: In GripperPlugin.cpp: loadDebugMode(): "
            << "missing nested <printDelayInSeconds> tag in <debug> tag, "
            << "defaulting to 3.0 seconds");
          debugUpdatePeriodInSeconds = 3.0;
        }

        ros::console::levels::Level dLevel = ros::console::levels::Debug;

        if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, dLevel)) {
          ros::console::notifyLoggerLevelsChanged();
        }
      } else if(debugString.compare("false") != 0) {
        ROS_INFO_STREAM("[Gripper Plugin : " << model->GetName()
          << "]: In GripperPlugin.cpp: loadDebugMode(): "
          << "invalid value in <printToConsole> tag in <debug> tag, "
          << "printToConsole = " << debugString << ", defaulting to false");
      }
    } else {
      ROS_INFO_STREAM("[Gripper Plugin : " << model->GetName()
        << "]: In GripperPlugin.cpp: loadDebugMode(): "
        << "missing nested <printToConsole> tag in <debug> tag, "
        << "defaulting to false");
    }
  } else {
    ROS_INFO_STREAM("[Gripper Plugin : " << model->GetName()
      << "]: In GripperPlugin.cpp: loadDebugMode(): "
      << "missing <debug> tag, defaulting to false");
  }
}

/**
 * This function loads the update rate from the SDF configuration file and uses
 * that value to set the update period. Effectively, the updatePeriod variable
 * defines how many times per second the plugin will apply changes from the ROS
 * subscribers. This value also determines the rate that debug statements will
 * be printed to the console if debugging mode is active.
 */
void GripperPlugin::loadUpdatePeriod() {
  float updateRate;

  if(!sdf->HasElement("updateRate")) {
    ROS_INFO_STREAM("[Gripper Plugin : " << model->GetName()
      << "]: In GripperPlugin.cpp: loadUpdatePeriod(): "
      << "missing <updateRate> tag, defaulting to 1000.0");
    updateRate = 1000.0;
  } else {
    updateRate = sdf->GetElement("updateRate")->Get<float>();

    // fatal error: the update cannot be <= 0 and especially cannot = 0
    if(updateRate <= 0) {
      ROS_ERROR_STREAM("[Gripper Plugin : " << model->GetName()
        << "]: In GripperPlugin.cpp: loadUpdatePeriod(): "
        << "updateRate = " << updateRate << ", updateRate cannot be <= 0.0");
      exit(1);
    }
  }

  // set the update period for this plugin: the plugin will refresh at a rate
  // of "updateRate" times per second, i.e., at "updatePeriodInSeconds" hertz
  updatePeriodInSeconds = 1.0 / updateRate;
}

/**
 * This function loads a string used for a subscription topic for this plugin.
 * Currently, this plugin will use two subscribers: one for the wrist, and one
 * for both finger joints.
 *
 * @param topicTag A std::string representing a tag in the SDF configuration
 *                 file for this plugin where a topic is defined.
 * @return A std::string containing the topic defined within the supplied tag.
 */
std::string GripperPlugin::loadSubscriptionTopic(std::string topicTag) {
  string topic;

  if(sdf->HasElement(topicTag)) {
    topic = sdf->GetElement(topicTag)->Get<std::string>();
  } else {
    ROS_ERROR_STREAM("[Gripper Plugin : " << model->GetName()
      << "]: In GripperPlugin.cpp: loadSubscriptionTopic(): No <" << topicTag
      << "> tag is defined in the model SDF file");
    exit(1);
  }

  return topic;
}

/**
 * This function loads a joint specified in an SDF configuration file. All
 * joints that are loaded are required for the plugin to work properly.
 * Therefore, in the event that a joint isn't properly loaded, exit(1) is
 * called and ROS is shutdown and/or crashed as a result.
 *
 * @param jointTag A string representing the name of the tag in the SDF file
 *                 where a joint is defined.
 * @return One of the gripper's three joints: wrist, left finger, or right
 *         finger.
 */
physics::JointPtr GripperPlugin::loadJoint(std::string jointTag) {
  string jointName;
  physics::JointPtr joint;
	    
  if(sdf->HasElement(jointTag)) {
    jointName = sdf->GetElement(jointTag)->Get<std::string>();
    joint = this->model->GetJoint(jointName);
		
    if(!joint) {
      ROS_ERROR_STREAM("[Gripper Plugin : " << model->GetName()
        << "]: In GripperPlugin.cpp: loadJoint(): No " << jointName
        << " joint is defined in the model SDF file");
      exit(1);
    }
  } else {
      ROS_ERROR_STREAM("[Gripper Plugin : " << model->GetName()
        << "]: In GripperPlugin.cpp: loadJoint(): No <" << jointTag
        << "> tag is defined in the model SDF file");
      exit(1);
  }

  return joint;
}

/**
 * This function loads (optional) user definable settings for the PID
 * controllers for the gripper wrist and finger joints. If the tags are not
 * defined or defined improperly, a set of default values will be used.
 *
 * @param PIDTag Defines which joint(s) we will load settings for. The only two
 *               valid values for PIDTag = "wrist" or "finger".
 * @return A PIDSettings struct with all values initialized based on user input
 *         from an SDF file or the predefined defaults within this code.
 */
PIDController::PIDSettings GripperPlugin::loadPIDSettings(string PIDTag) {
  if(PIDTag.compare("wrist") != 0 && PIDTag.compare("finger") != 0) {
    ROS_ERROR_STREAM("[Gripper Plugin : " << model->GetName()
      << "]: In GripperPlugin.cpp: loadPIDSettings(): PIDTag " << PIDTag
      << " is invalid: use either \"wrist\" or \"finger\"");
    exit(1);
  }

  PIDController::PIDSettings settings;
  math::Vector3 pid;
  math::Vector2d forceLimits;

  if(!sdf->HasElement(PIDTag + "PID")) {
    ROS_INFO_STREAM("[Gripper Plugin : " << model->GetName()
      << "]: In GripperPlugin.cpp: loadPIDSettings(): missing <" << PIDTag
      << "PID> tag, defaulting to P=2.5, I=0.0, D=0.0");
    pid = math::Vector3(2.5, 0.0, 0.0);
  } else {
    pid = sdf->GetElement(PIDTag + "PID")->Get<math::Vector3>();
  }

  if(!sdf->HasElement(PIDTag + "ForceLimits")) {
    ROS_INFO_STREAM("[Gripper Plugin : " << model->GetName()
      << "]: In GripperPlugin.cpp: loadPIDSettings(): missing <" << PIDTag
      << "ForceLimits> tag, defaulting to MIN = -10.0 N, MAX = 10.0 N");
    forceLimits = math::Vector2d(-10.0, 10.0);
  } else {
    forceLimits =
      sdf->GetElement(PIDTag + "ForceLimits")->Get<math::Vector2d>();
  }

  settings.Kp  = (float)pid.x;
  settings.Ki  = (float)pid.y;
  settings.Kd  = (float)pid.z;
  settings.dt  = updatePeriodInSeconds;
  settings.min = (float)forceLimits.x;
  settings.max = (float)forceLimits.y;

  return settings;
}

/**
 */
void GripperPlugin::handleGrasping() {

  bool inContact = false;

  // Get the amount of time that elapsed between this call and the
  // last time we were called. This is used for the contact and no contact
  // time calculations
  common::Time currentTime = model->GetWorld()->GetSimTime();
  common::Time deltaTime = (currentTime - prevHandleGraspingTime);
  prevHandleGraspingTime = currentTime;
  
  // The following conditionals check to see whether we should attach the
  // gripper to a target

  // Add to finger contact timers. These values are reset to zero by the
  // finger contact handlers whenever a contact with a targets occurs. If these
  // timers reach the timeout threshold the target may be dropped.
  rightFingerNoContactTime += deltaTime;
  leftFingerNoContactTime += deltaTime;

  // Check whether both fingers are in contact with a target
  if ( rightFingerTargetLink && leftFingerTargetLink )
      
    // Check whether the right and left fingers are in contact with the
    // same object
    if (rightFingerTargetLink == leftFingerTargetLink) {
      
      // Check how long the fingers have been in contact with the object
      // if longer than contactThreshold then attach
      noContactTime = 0;
      contactTime += deltaTime;
      if ( contactTime > contactThreshold ) {
        inContact = true;
        if (!isAttached) 
          try {
            attach();  
          } catch (exception &e) {
            sendInfoLogMessage("GripperPlugin: attach() failed with: " + string(e.what()));
          }
      }
    }
 
  // Check whether we should detach an existing gripper-target joint
  // Detach whenever the attach criteria above are not met for longer
  // than the noContactThreshold.
  if ( !rightFingerTargetLink && !leftFingerTargetLink )
  if (!inContact) {
    noContactTime += deltaTime;
    contactTime = 0;
    if ( noContactTime > noContactThreshold) {
      
      // Only detach if attached
      if (isAttached)
        try {
	  sendInfoLogMessage("Attempting detach...");
          detach();
        } catch (exception &e) {
          sendInfoLogMessage("GripperPlugin: handleGrasping() failed with: " + string(e.what()));
        }
    }
  }
  
  // Fail safe - sometimes the cubes get stuck. Always detach if the finger angle is wide enough  
  if (desiredFingerAngle >= maxGrippingAngle) {
      if (isAttached)
      try {
	sendInfoLogMessage("GripperPlugin: handleGrasping() trying to detach due to finger angle failsafe. Finger angle is "+ to_string(desiredFingerAngle.Degree()) + ", max gripping angle is " + to_string(maxGrippingAngle.Degree()));
        detach(); 
      } catch (exception &e) {
	  sendInfoLogMessage("GripperPlugin: handleGrasping() failed with: " + string(e.what()));
      }
  }
}

/**
 */
void GripperPlugin::attach() {

  lock_guard<std::mutex> lock(attaching_mutex);
  
  if (!rightFingerTargetLink || !leftFingerTargetLink) throw runtime_error("NULL target link. Aborting attach.");

  if (isAttached) throw runtime_error("already attached");

  // Do not attach in the edge case where the gripper fingers are fully open.
  // Gripping will occur once the rover begins to grasp and the desiredFingerAngle is lowered.
  if (desiredFingerAngle >= maxGrippingAngle) return;

  // Get the target model
  physics::ModelPtr targetModel = rightFingerTargetLink->GetModel(); // It doesn't matter whether we use the left or right target link here.
  attachedTargetModel = targetModel;

   // If the model is dynamic create a joint between the gripper and the target link. If the model is not static attach the model to the gripper as a static model
  if (!attachedTargetModel->IsStatic())
  {
  
    // Create a new joint with which to connect the target and gripper
    targetAttachJoint = model->GetWorld()->GetPhysicsEngine()->CreateJoint("revolute");
    targetAttachJoint->SetName(model->GetName()+"_gripper_attach_joint");
    targetAttachJoint->Load(rightFingerTargetLink, gripperAttachLink, math::Pose(rightFingerTargetLink->GetWorldPose().pos, math::Quaternion()));
    targetAttachJoint->Attach(gripperAttachLink, rightFingerTargetLink);
    
    // set the axis of revolution
    math::Vector3 axis(0,0,1);
    targetAttachJoint->SetAxis(0, axis);
  
  // Initialize the joint so it doesn't move too much
  // The dynamics of the target grip can be controlled here
  double cfm, erp; // Constrained force mixing and Error Reduction parameter
  double dt = model->GetWorld()->GetPhysicsEngine()->GetMaxStepSize();
  if (dt < 1e-6) dt = 1e-6;
  double stiffness = 20000.0f;
  double damping = 100.0f;
  erp = stiffness*dt / (stiffness*dt + damping);
  cfm = 1.0 / (stiffness*dt + damping);
  targetAttachJoint->SetParam("erp", 0, erp);
  targetAttachJoint->SetParam("cfm", 0, cfm);
  targetAttachJoint->SetParam("stop_erp", 0, erp);
  targetAttachJoint->SetParam("stop_cfm", 0, cfm);
  targetAttachJoint->SetHighStop(0, 0.0);
  targetAttachJoint->SetLowStop(0, 0.0);

  } else { // The target model we are trying to grasp is static.

    if (!targetModel.get()){
      string errorMsg = "Model " + targetModel->GetName()  + " could not be found";
      throw runtime_error(errorMsg);
     }

    attachedTargetOffset = targetModel->GetWorldPose() - gripperAttachLink->GetWorldPose();
    
    attachedTargetModel = targetModel;
  }

  isAttached = true;

  stringstream poseDebugSStr;

  sendInfoLogMessage("Gripper attached to "
                     + (targetModel->IsStatic() ?
                        string(" static target ") :
                        string(" dynamic target "))
                     + targetModel->GetName()
                     + " after being in contact for "
                     + to_string(contactTime.Double())
                     );
 
 
}

/**
 */
void GripperPlugin::detach() {
  lock_guard<std::mutex> lock(attaching_mutex);

  if (!isAttached) throw runtime_error("not attached");
  
  if (!attachedTargetModel) {
    string errorMsg = "Attached target model is NULL";
    throw runtime_error(errorMsg);
  }

  if (!attachedTargetModel.get()){
    string errorMsg = "Model " + attachedTargetModel->GetName()  + " could not be found";
    throw runtime_error(errorMsg);
  }
 
  if (!attachedTargetModel->IsStatic()) {

    stringstream poseStream;
    poseStream << attachedTargetModel->GetWorldPose();
    sendInfoLogMessage("Gripper detached from "
                       + (attachedTargetModel->IsStatic()? string("static"): string("dynamic"))
                       + " model "
                       + attachedTargetModel->GetName() 
                       + " after no contact for " 
                       + to_string(noContactTime.Double())
                       + ". Target end pose: " + poseStream.str());
    
    targetAttachJoint->Detach();
    targetAttachJoint.reset();
    isAttached = false;
    attachedTargetModel = NULL;
    contactTime = common::Time(0.0);
    
    return;
    
  } else {
    // Drop the target to the ground. The drop placement is handled by
    // the update static target pose function, as is finalizing the
    // attached model state.
    dropStaticTarget = true;
  }
}

// Contact handlers are triggered by contact with the gripper fingers.
// Finds the link that the finger is in contact with and
// sets the class link variable for use by the handleGrasp() function.
// Set the link pointer to NULL if the finger is not in contact
// with a target object.
// Since the collision involves two objects and we don't know which might
// be a target object we have to check collision1 and collision2.
void GripperPlugin::rightFingerContactEventHandler(ConstContactsPtr& msg){
  if (attaching_mutex.try_lock()){
    lock_guard<mutex> lock(attaching_mutex, adopt_lock_t());
    
    for(unsigned int i=0; i < msg->contact_size(); i++){
      string name1 = msg->contact(i).collision1();
      string name2 = msg->contact(i).collision2();
      
      // Parse the collision name to find the link name (approporate GetChildLink accerros not available) This is a hacky way around that.
      string collision1ModelName = name1.substr(0, name1.find("::"));
      string collision2ModelName = name2.substr(0, name2.find("::"));
      
      physics::ModelPtr modelInCollision;
      
      if (collision1ModelName.substr(0,2).compare("at")==0) {
        modelInCollision = model->GetWorld()->GetModel(collision1ModelName);
        rightFingerTargetLink = modelInCollision->GetLink("link");
	rightFingerNoContactTime = 0.0f;
        return;
      } else if (collision2ModelName.substr(0,2).compare("at")==0) {
        modelInCollision = model->GetWorld()->GetModel(collision2ModelName);
        rightFingerTargetLink = modelInCollision->GetLink("link");
	rightFingerNoContactTime = 0.0f;
        return;
      }
    }
    
    if (rightFingerNoContactTime > fingerNoContactThreshold)
      rightFingerTargetLink = NULL;
  }
}

void GripperPlugin::leftFingerContactEventHandler(ConstContactsPtr& msg){
  if (attaching_mutex.try_lock()){
    lock_guard<mutex> lock(attaching_mutex, adopt_lock_t());
    
    for(unsigned int i=0; i < msg->contact_size(); i++){
      string name1 = msg->contact(i).collision1();
      string name2 = msg->contact(i).collision2();
      physics::ModelPtr modelInCollision;
      
      // Parse the collision name to find the link name (approporate GetChildLink accerros not available) This is a hacky way around that.
      string collision1ModelName = name1.substr(0, name1.find("::"));
      string collision2ModelName = name2.substr(0, name2.find("::"));
      
      if (collision1ModelName.substr(0,2).compare("at")==0) {
        modelInCollision = model->GetWorld()->GetModel(collision1ModelName);
        leftFingerTargetLink = modelInCollision->GetLink("link");
	leftFingerNoContactTime = 0.0f;
        return;
      } else if (collision2ModelName.substr(0,2).compare("at")==0) {
        modelInCollision = model->GetWorld()->GetModel(collision2ModelName);
        leftFingerTargetLink = modelInCollision->GetLink("link");
	leftFingerNoContactTime = 0.0f;
        return;
      }
    }
    
    if (leftFingerNoContactTime > fingerNoContactThreshold)
      leftFingerTargetLink = NULL;
  }
}

void GripperPlugin::sendInfoLogMessage(string text) {
 std_msgs::String msg;
 msg.data = model->GetName() + ": " + text;
 infoLogPublisher.publish(msg);
}


void GripperPlugin::updateGraspedStaticTargetPose() {
  
  // We don't want to update the position of the target while it is being
  // detached
  // Try the lock and do nothing if it is locked
  if (attaching_mutex.try_lock()){  
    lock_guard<mutex> lock(attaching_mutex, adopt_lock_t());
    
    // Is the gripper grasping something we need to move
    if (!isAttached) return;
    
    // Make sure the attached model pointer is non NULL
    if (!attachedTargetModel) return;
    
    // This isn't needed for non-static grasped targets
    if (!attachedTargetModel->IsStatic()) return; 
    
    attachedTargetModel->SetWorldPose(attachedTargetOffset+gripperAttachLink->GetWorldPose());

    if (dropStaticTarget) {
      math::Pose p = attachedTargetModel->GetWorldPose();
      
      // Modify the position of the target so that its center is half the target height
      // above the ground. This should make the bottom flush with the ground.
      // Gazebo provides a convenient helper function for this.
      p.rot = math::Quaternion(1,0,0,0);
      attachedTargetModel->SetWorldPose(p,true);
      attachedTargetModel->PlaceOnNearestEntityBelow();
            
      if ( dropStaticTargetCounter++ > 100 )
      {
        isAttached = false;
        attachedTargetModel = NULL;
        dropStaticTarget = false;
        dropStaticTargetCounter = 0;
      }
    }
  }
}


GripperPlugin::~GripperPlugin() {
  
  rosNode->shutdown(); // Shutdown the ROS node

  // Stop the multi threaded ROS spinner
  gazebo::shutdown();
}
