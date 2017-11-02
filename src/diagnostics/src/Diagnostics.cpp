#include "Diagnostics.h"

#include <string>
#include <usb.h>
#include <sys/stat.h> // To check if a file exists
#include <std_msgs/String.h> // For creating ROS string messages
#include <ctime> // For time()

using namespace std;
using namespace gazebo;

Diagnostics::Diagnostics(std::string name) {

  node_heartbeat_timeout = 5.0;
  device_heartbeat_timeout = 2.0;
  diagnostics_start_time = ros::Time::now();
  node_start_delay = 20;

  this->publishedName = name;
  diagLogPublisher = nodeHandle.advertise<std_msgs::String>("/diagsLog", 1, true);
  diagnosticDataPublisher  = nodeHandle.advertise<std_msgs::Float32MultiArray>("/"+publishedName+"/diagnostics", 10);
  fingerAngleSubscribe = nodeHandle.subscribe(publishedName + "/fingerAngle/prev_cmd", 10, &Diagnostics::fingerTimestampUpdate, this);
  wristAngleSubscribe = nodeHandle.subscribe(publishedName + "/fingerAngle/prev_cmd", 10, &Diagnostics::wristTimestampUpdate, this);
  imuSubscribe = nodeHandle.subscribe(publishedName + "/imu", 10, &Diagnostics::imuTimestampUpdate, this);
  odometrySubscribe = nodeHandle.subscribe(publishedName + "/odom", 10, &Diagnostics::odometryTimestampUpdate, this);
  sonarLeftSubscribe = nodeHandle.subscribe(publishedName + "/sonarLeft", 10, &Diagnostics::sonarLeftTimestampUpdate, this);
  sonarCenterSubscribe = nodeHandle.subscribe(publishedName + "/sonarCenter", 10, &Diagnostics::sonarCenterTimestampUpdate, this);
  sonarRightSubscribe = nodeHandle.subscribe(publishedName + "/sonarRight", 10, &Diagnostics::sonarRightTimestampUpdate, this);
  abdridgeNodeSubscribe = nodeHandle.subscribe(publishedName + "/abridge/heartbeat", 1, &Diagnostics::abridgeNode,this);
  sbdridgeNodeSubscribe = nodeHandle.subscribe(publishedName + "/sbridge/heartbeat", 1, &Diagnostics::sbridgeNode,this);
  behaviourNodeSubscribe = nodeHandle.subscribe(publishedName + "/behaviour/heartbeat", 1, &Diagnostics::behaviourNode,this);
  ubloxNodeSubscribe = nodeHandle.subscribe(publishedName + "/fix" , 1, &Diagnostics::ubloxNode,this);

  // Initialize the variables we use to track the simulation update rate
  prevRealTime = common::Time(0.0);
  prevSimTime = common::Time(0.0);
  simRate = 0.0f;

  // Setup sensor check timers
  sensorCheckTimer = nodeHandle.createTimer(ros::Duration(sensorCheckInterval), &Diagnostics::sensorCheckTimerEventHandler, this);

  // Setup Node check timer
  nodeCheckTimer = nodeHandle.createTimer(ros::Duration(nodeCheckInterval), &Diagnostics::nodeCheckTimerEventHandler, this);
  
  simCheckTimer = nodeHandle.createTimer(ros::Duration(sensorCheckInterval), &Diagnostics::simCheckTimerEventHandler, this);

  if ( checkIfSimulatedRover() ) {
    // For processing gazebo messages from the world stats topic.
    // Used to gather information for simulated rovers

    // Create fake argv and argc for the gazebo setup
    char  arg0[] = "diagnostics";
    char* argv[] = { &arg0[0], NULL };
    int   argc   = (int)(sizeof(argv) / sizeof(argv[0])) - 1;
    gazebo::client::setup(argc, argv);

    // Create Gazebo node and init
    gazebo::transport::NodePtr newNode(new gazebo::transport::Node());
    gazeboNode = newNode;
    gazeboNode->Init();
    string worldStatsTopic = "/gazebo/default/world_stats";
    worldStatsSubscriber = gazeboNode->Subscribe(worldStatsTopic, &Diagnostics::simWorldStatsEventHandler, this);
 
    simulated = true;
    publishInfoLogMessage("Diagnostic Package Started. Simulated Rover.");
  } else {
    simulated = false;
    publishInfoLogMessage("Diagnostic Package Started. Physical Rover. ");
    
    try {       
      string name = wirelessDiags.setInterface();
      publishInfoLogMessage("Monitoring wireless interface " + name);
    } catch( exception &e ) {
      publishErrorLogMessage("Error setting interface name for wireless diagnostics: " + string(e.what()));
    }
  }
}

void Diagnostics::publishDiagnosticData() {
  if (!simulated) {
  WirelessInfo info;

  // Get info about the wireless interface
  // Catch and display an error if there was an exception
  try {
    info = wirelessDiags.getInfo();
  } catch( exception &e ){
    publishErrorLogMessage(e.what());
  return;
}

  std_msgs::Float32MultiArray rosMsg;
  rosMsg.data.clear();
  rosMsg.data.push_back(info.quality);
  rosMsg.data.push_back(info.bandwidthUsed);
  rosMsg.data.push_back(-1); // Sim update rate
  diagnosticDataPublisher.publish(rosMsg);  
  }
}

void Diagnostics::publishErrorLogMessage(std::string msg) {

  std_msgs::String ros_msg;
  ros_msg.data = "<font color=Red size=3>" + publishedName+" ("+getHumanFriendlyTime()+"): " + msg + "</font>";
  diagLogPublisher.publish(ros_msg);

}

void Diagnostics::publishWarningLogMessage(std::string msg) {
  std_msgs::String ros_msg;
  ros_msg.data = "<font color=Yellow size=2>" + publishedName+" ("+getHumanFriendlyTime()+"): " + msg + "</font>";
  diagLogPublisher.publish(ros_msg);
}

void Diagnostics::publishInfoLogMessage(std::string msg) {
  std_msgs::String ros_msg;
  ros_msg.data = "<font color=Lime size=2>" + publishedName + " ("+getHumanFriendlyTime()+"): " + msg + "</font>";
  diagLogPublisher.publish(ros_msg);
}

void Diagnostics::fingerTimestampUpdate(const geometry_msgs::QuaternionStamped::ConstPtr& message) {
	fingersTimestamp = message->header.stamp;
}

void Diagnostics::wristTimestampUpdate(const geometry_msgs::QuaternionStamped::ConstPtr& message) {
	wristTimestamp = message->header.stamp;
}

void Diagnostics::imuTimestampUpdate(const sensor_msgs::Imu::ConstPtr& message) {
	imuTimestamp = message->header.stamp;
}

void Diagnostics::odometryTimestampUpdate(const nav_msgs::Odometry::ConstPtr& message) {
	odometryTimestamp = message->header.stamp;
}

void Diagnostics::sonarLeftTimestampUpdate(const sensor_msgs::Range::ConstPtr& message) {
	sonarLeftTimestamp = message->header.stamp;
}

void Diagnostics::sonarCenterTimestampUpdate(const sensor_msgs::Range::ConstPtr& message) {
	sonarCenterTimestamp = message->header.stamp;
}

void Diagnostics::sonarRightTimestampUpdate(const sensor_msgs::Range::ConstPtr& message) {
    sonarRightTimestamp = message->header.stamp;
}

void Diagnostics::abridgeNode(std_msgs::String msg) {
    abridgeNodeTimestamp = ros::Time::now();
}

void Diagnostics::sbridgeNode(std_msgs::String msg) {
    sbridgeNodeTimestamp = ros::Time::now();
}

void Diagnostics::behaviourNode(std_msgs::String msg) {
    behaviourNodeTimestamp = ros::Time::now();
}

void Diagnostics::ubloxNode(const sensor_msgs::NavSatFix::ConstPtr& message) {
    ubloxNodeTimestamp = ros::Time::now();
}

// Return the current time in this timezone in "WeekDay Month Day hr:mni:sec year" format.
// We use this instead of asctime or ctime because it is thread safe
string Diagnostics::getHumanFriendlyTime() {
   time_t t = std::time(NULL);
   char humanReadableStr[100];
   
   if (strftime(humanReadableStr, sizeof(humanReadableStr), "%A %c", localtime(&t)))
     return humanReadableStr;
   else
     return ""; // There was a problem getting the time. Return the empty string.
   
}

// sensor check timeout handler. This function is triggered periodically and calls the
// sensor check functions.
void Diagnostics::sensorCheckTimerEventHandler(const ros::TimerEvent& event) {

  if (!simulated) {
  checkIMU();
  checkGPS();
  checkSonar();
  checkCamera();
  checkGripper();
  checkOdometry();
  
  publishDiagnosticData();
  }

}

void Diagnostics::nodeCheckTimerEventHandler(const ros::TimerEvent& event) {


    if (node_start_delay > (ros::Time::now() - diagnostics_start_time).sec) return;

    if (!simulated) {
        checkAbridge();
        checkUblox();
    }
    else {
       checkSbridge();
    }

    checkBehaviour();

}

void Diagnostics::simCheckTimerEventHandler(const ros::TimerEvent& event) {
  
  if (simulated) {
    std_msgs::Float32MultiArray rosMsg;
    rosMsg.data.clear();
    rosMsg.data.push_back(0.0f);
 rosMsg.data.push_back(0.0f);
    rosMsg.data.push_back(checkSimRate());
    diagnosticDataPublisher.publish(rosMsg);
  }
  
}

float Diagnostics::checkSimRate() {
  return simRate;
}

void Diagnostics::checkIMU() {
  // Example
  //publishWarningLogMessage("IMU Warning");
    if (ros::Time::now() - imuTimestamp <= ros::Duration(device_heartbeat_timeout)) {
		if (!imuConnected) {
			imuConnected = true;
			publishInfoLogMessage("IMU connected");
		}
	}
	else if (imuConnected) {
		imuConnected = false;
		publishErrorLogMessage("IMU is not connected");
	}
}

void Diagnostics::checkGPS() {
  // Example
  //publishWarningLogMessage("GPS Warning");

  // Check that a U-Blox device exists in the connected USB devices list

  if ( checkGPSExists() ) {
    // Notify the GUI only if reconnected after being previously disconnected
    if (!GPSConnected) publishInfoLogMessage("GPS reconnected");
    GPSConnected = true;
  } else { 
    if (GPSConnected) // Guard against repeating the error.
      publishErrorLogMessage("GPS is not connected");
    GPSConnected = false;
  }
}

void Diagnostics::checkCamera() {
    // Example
  //publishWarningLogMessage("Camera Warning");

  // Check that a Logitec c170 device exists in the connected USB devices list
  if ( checkCameraExists() ) {
    // Notify the GUI only if reconnected after being previously disconnected
    if (!cameraConnected) publishInfoLogMessage("Camera reconnected");
    cameraConnected = true;
  } else {
    if (cameraConnected)
    publishErrorLogMessage("Camera not connected");
    cameraConnected = false;
  }
}

void Diagnostics::checkSonar() {
  //Example
  //publishErrorLogMessage("Sonar Error");

    if (ros::Time::now() - sonarLeftTimestamp <= ros::Duration(device_heartbeat_timeout)) {
		if (!sonarLeftConnected) {
			sonarLeftConnected = true;
			publishInfoLogMessage("Left ultrasound connected");
		}
	}
	else if (sonarLeftConnected) {
		sonarLeftConnected = false;
		publishErrorLogMessage("Left ultrasound is not connected");
	}

    if (ros::Time::now() - sonarCenterTimestamp <= ros::Duration(device_heartbeat_timeout)) {
		if (!sonarCenterConnected) {
			sonarCenterConnected = true;
			publishInfoLogMessage("Center ultrasound connected");
		}
	}
	else if (sonarCenterConnected) {
		sonarCenterConnected = false;
		publishErrorLogMessage("Center ultrasound is not connected");
	}

    if (ros::Time::now() - sonarRightTimestamp <= ros::Duration(device_heartbeat_timeout)) {
		if (!sonarRightConnected) {
			sonarRightConnected = true;
			publishInfoLogMessage("Right ultrasound connected");
		}
	}
	else if (sonarRightConnected) {
		sonarRightConnected = false;
		publishErrorLogMessage("Right ultrasound is not connected");
	}
}

void Diagnostics::checkGripper() {
	// Example
	//publishWarningLogMessage("Gripper Warning");

    if (ros::Time::now() - fingersTimestamp <= ros::Duration(device_heartbeat_timeout)) {
		if (!fingersConnected) {
			fingersConnected = true;
			publishInfoLogMessage("Gripper fingers connected");
		}
	}
	else if (fingersConnected) {
		fingersConnected = false;
		publishErrorLogMessage("Gripper fingers are not connected");
	}

    if (ros::Time::now() - wristTimestamp <= ros::Duration(device_heartbeat_timeout)) {
		if (!wristConnected) {
			wristConnected = true;
			publishInfoLogMessage("Gripper wrist connected");
		}
	}
	else if (wristConnected) {
		wristConnected = false;
		publishErrorLogMessage("Gripper wrist is not connected");
	}
}

void Diagnostics::checkOdometry() {
	// Example
	//publishWarningLogMessage("Odometry Warning");

    if (ros::Time::now() - odometryTimestamp <= ros::Duration(device_heartbeat_timeout)) {
		if (!odometryConnected) {
			odometryConnected = true;
			publishInfoLogMessage("Encoders connected");
		}
	}
	else if (odometryConnected) {
		odometryConnected = false;
		publishErrorLogMessage("Encoders are not connected");
	}
}

void Diagnostics::checkAbridge() {

    if (ros::Time::now() - abridgeNodeTimestamp <= ros::Duration(node_heartbeat_timeout)) {
        if (!abridgeRunning) {
            abridgeRunning = true;
            publishInfoLogMessage("the abridge node is now running");
        }
    }
    else if (abridgeRunning) {
        abridgeRunning = false;
        publishErrorLogMessage("the abridge node is not running");
    }
}

void Diagnostics::checkSbridge() {

    if (ros::Time::now() - sbridgeNodeTimestamp <= ros::Duration(node_heartbeat_timeout)) {
        if (!sbridgeRunning) {
            sbridgeRunning = true;
            publishInfoLogMessage("the sbridge node is now running");
        }
    }
    else if (sbridgeRunning) {
        sbridgeRunning = false;
        publishErrorLogMessage("the sbridge node is not running");
    }
}

void Diagnostics::checkBehaviour() {

    if (ros::Time::now() - behaviourNodeTimestamp <= ros::Duration(node_heartbeat_timeout)) {
        if (!behaviourRunning) {
            behaviourRunning = true;
            publishInfoLogMessage("the behaviour node is now running");
        }
    }
    else if (behaviourRunning) {
        behaviourRunning = false;
        publishErrorLogMessage("the behaviour node is not running");
    }
}

void Diagnostics::checkUblox() {

    if (ros::Time::now() - ubloxNodeTimestamp <= ros::Duration(node_heartbeat_timeout)) {
        if (!ubloxRunning) {
            ubloxRunning = true;
            publishInfoLogMessage("the ublox node is now running");
        }
    }
    else if (ubloxRunning) {
        ubloxRunning = false;
        publishErrorLogMessage("the ublox node is not running");
    }
}

// Check if the U-Blox GPS is connected
// ID_VENDOR = 0x1546
// ID_PRODUCT = 0x01a6
bool Diagnostics::checkGPSExists(){
  uint16_t GPSVendorID = 0x1546;
  uint16_t GPSProductID = 0x01a6;
  return checkUSBDeviceExists( GPSVendorID, GPSProductID );
}

// Check if the Logitech c170 camera is connected
// ID_VENDOR = 0x046d
// ID_PRODUCT = 0x082b
bool Diagnostics::checkCameraExists(){
  uint16_t cameraVendorID = 0x046d;
  uint16_t cameraProductID = 0x082b;
  return checkUSBDeviceExists( cameraVendorID, cameraProductID );
}


// Search through the connected USB devices for one that matches the
// specified vendorID and productID
bool Diagnostics::checkUSBDeviceExists(uint16_t vendorID, uint16_t productID){
  
  struct usb_bus *bus;
  struct usb_device *dev;
  usb_init();
  usb_find_busses();
  usb_find_devices();

  // Iterate through busses and devices
  for (bus = usb_busses; bus; bus = bus->next)
    for (dev = bus->devices; dev; dev = dev->next)
      if ( dev->descriptor.idVendor == vendorID && dev->descriptor.idProduct == productID )
        return true; 

  // GPS not found
  return false;
}

void Diagnostics::simWorldStatsEventHandler(ConstWorldStatisticsPtr &msg) {
  
  const msgs::Time simTimeMsg = msg->sim_time();
  const msgs::Time realTimeMsg = msg->real_time();

  common::Time simTime(simTimeMsg.sec(), simTimeMsg.nsec());
  common::Time realTime(realTimeMsg.sec(), realTimeMsg.nsec());
  
  common::Time deltaSimTime = simTime - prevSimTime;
  common::Time deltaRealTime = realTime - prevRealTime;

  prevSimTime = simTime;
  prevRealTime = realTime;

  simRate = (deltaSimTime.Double())/(deltaRealTime.Double());
}

// Check whether a rover model file exists with the same name as this rover name
// if not then we should be a physcial rover. Need a better method.
bool Diagnostics::checkIfSimulatedRover() {
  struct stat buffer;
  const char *model_path_env = "GAZEBO_MODEL_PATH";
  char *model_root = getenv(model_path_env);
  string model_path = string(model_root)+"/"+publishedName+"/model.sdf";
  return (stat(model_path.c_str(), &buffer) == 0); 
}
     
Diagnostics::~Diagnostics() {
  gazebo::shutdown();
}

