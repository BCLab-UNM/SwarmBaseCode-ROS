#include "GripperPlugin.h"

using namespace gazebo;
using namespace std;

void GripperPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  model = _model;
  sdf = _sdf;

  std::cout << "[Gripper Plugin]: loading gripper plugin for " << model->GetName() << "\n";

  wristJoint = loadJoint("wristJoint");
  leftFingerJoint = loadJoint("leftFingerJoint");
  rightFingerJoint = loadJoint("rightFingerJoint");

  updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&GripperPlugin::updateWorldEventHandler, this));

  if (!ros::isInitialized()) {
    std::cout << "      [Gripper Plugin] initializing ros" << std::endl;
    int argc = 0;
    char **argv=NULL;
    ros::init(argc, argv, model->GetName() + "_GRIPPER", ros::init_options::NoSigintHandler);
  } else {
    std::cout << "      [Gripper Plugin] ROS already initialized." << std::endl;
  }

  rosNode.reset(new ros::NodeHandle(string(model->GetName()) + "_GRIPPER"));

  string wristTopic, fingerTopic;

  if(sdf->HasElement("wristTopic")) {
    wristTopic = sdf->GetElement("wristTopic")->Get<std::string>();
  } else {
    cout << "Error: In GripperPlugin.cpp: Load(): No <wristTopic> tag defined in model sdf file" << endl;
    exit(1);
  }

  if(sdf->HasElement("fingerTopic")) {
    fingerTopic = sdf->GetElement("fingerTopic")->Get<std::string>();
  } else {
    cout << "Error: In GripperPlugin.cpp: Load(): No <fingerTopic> tag defined in model sdf file" << endl;
    exit(1);
  }

  // subscribe to ROS topics - begin
  ros::SubscribeOptions wristSubscriptionOptions =
    ros::SubscribeOptions::create<std_msgs::Float32>(wristTopic, 1, boost::bind(&GripperPlugin::setWristAngleHandler, this, _1), ros::VoidPtr(), &rosQueue);
				
  wristAngleSubscriber = rosNode->subscribe(wristSubscriptionOptions);

  ros::SubscribeOptions fingerSubscriptionOptions =
    ros::SubscribeOptions::create<std_msgs::Float32>(fingerTopic, 1, boost::bind(&GripperPlugin::setFingerAngleHandler, this, _1), ros::VoidPtr(), &rosQueue);
				
  fingerAngleSubscriber = rosNode->subscribe(fingerSubscriptionOptions);
  // subscribe to ROS topics - end

  // Spin up the queue helper thread
  rosQueueThread = std::thread(std::bind(&GripperPlugin::processRosQueue, this));

  std::cout << "[Gripper Plugin]: loading complete for " << model->GetName() << "\n";	
}

void GripperPlugin::updateWorldEventHandler() {

  GripperState currentState;
  GripperState desiredState;
    
  // Set the current gripper state
  currentState.wristAngle = wristJoint->GetAngle(0).Radian();
  currentState.leftFingerAngle = leftFingerJoint->GetAngle(0).Radian();
  currentState.rightFingerAngle = rightFingerJoint->GetAngle(0).Radian();

  // Get the total angle of both fingers,
  // the right finger joint angle is always negative
  // the left finger joint angle is always positive
  // The total finger angle = left finger joint angle - ( -right finger joint angle )
  // the total finger angle is ALWAYS POSITIVE (or zero)
  float leftFingerAngle = leftFingerJoint->GetAngle(0).Radian();
  float rightFingerAngle = rightFingerJoint->GetAngle(0).Radian();

  // Set the desired gripper state
  desiredState.leftFingerAngle = desiredFingerAngle.Radian() / 2.0;
  desiredState.rightFingerAngle = -desiredFingerAngle.Radian() / 2.0;
  desiredState.wristAngle = desiredWristAngle.Radian();
  
  // Get the forces to apply to the joints from the PID controllers
  GripperForces commandForces = gripperManager.getForces(desiredState, currentState);

  //cout << "[Gripper Plugin]: Current State: Wrist Angle: " << currentState.wristAngle << ", " << " L Finger Angle: " << currentState.leftFingerAngle << " R Finger Angle: " << currentState.rightFingerAngle << endl;  
  //cout << "[Gripper Plugin]: Desired State: Wrist Angle: " << desiredState.wristAngle << ", " << " L Finger Angle: " << desiredState.leftFingerAngle << " R Finger Angle: " << desiredState.rightFingerAngle << endl;  
  //cout << "[Gripper Plugin]: Applying Forces:  Wrist: " << commandForces.wristForce << " N, L Finger Force: " << commandForces.leftFingerForce << " N, " << "R Finger Force: " << commandForces.rightFingerForce << " N" << endl;  


  // Apply the command forces to the joints
  wristJoint->SetForce(0, commandForces.wristForce);
  leftFingerJoint->SetForce(0, commandForces.leftFingerForce);
  rightFingerJoint->SetForce(0, commandForces.rightFingerForce);
}

void GripperPlugin::setWristAngleHandler(const std_msgs::Float32ConstPtr& msg) {
	    float wristAngle = msg->data;
	    //std::cout << model->GetName() << ": wristAngle message received: " <<  wristAngle << std::endl;
	    desiredWristAngle = wristAngle;
}

void GripperPlugin::setFingerAngleHandler(const std_msgs::Float32ConstPtr& msg) {
	    float fingerAngle = msg->data;
	    //std::cout << model->GetName() << ": fingerAngle message received: " <<  fingerAngle << std::endl;
	    desiredFingerAngle = fingerAngle;
}

	  
	  
	  /**
	   * This function loads a joint and joint name for the specified elements.
	   * During the current development phase of this plugin, message output is
	   * especially verbose to facilitate debugging.
	   */
physics::JointPtr GripperPlugin::loadJoint(std::string jointTag)
	  {
	    std::string jointName;
	    physics::JointPtr joint;
	    
	    if(sdf->HasElement(jointTag))
	      {
		std::cout << "      [Gripper Plugin]: found <" << jointTag
			  << "> tag in SDF\n";
		
		jointName = sdf->GetElement(jointTag)->Get<std::string>();
		joint = this->model->GetJoint(jointName);
		
		if(!joint)
		  {
		    std::cout << "      [Gripper Plugin]: ERROR! " << jointName
			      << " is undefined for <" << jointTag << ">\n";
		  }
		else
		  {
		    std::cout << "      [Gripper Plugin]: " << jointName
			      << " loaded successfully for <" << jointTag << ">\n";
		  }
	      }
	    else
				{
				  std::cout << "      [Gripper Plugin]: ERROR! missing <" << jointTag
					    << "> tag in SDF\n";
				}
	    
	    return joint;
	  }
	  

void GripperPlugin::processRosQueue() {
	    static const double timeout = .01;
	    while (this->rosNode->ok()) {
	      this->rosQueue.callAvailable(ros::WallDuration(timeout));
	    }
}
