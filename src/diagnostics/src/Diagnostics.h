#ifndef Diagnostics_h
#define Diagnostics_h

#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <ros/ros.h>

#include <geometry_msgs/QuaternionStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>

#include "WirelessDiags.h"

// The following multiarray headers are for the diagnostics data publisher
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"

#include <string>
#include <exception>

class Diagnostics {
  
public:
  Diagnostics(std::string);
  ~Diagnostics();
  void publishWarningLogMessage(std::string);
  void publishErrorLogMessage(std::string);
  void publishInfoLogMessage(std::string);
  
  void fingerTimestampUpdate(const geometry_msgs::QuaternionStamped::ConstPtr& message);
  void wristTimestampUpdate(const geometry_msgs::QuaternionStamped::ConstPtr& message);
  void imuTimestampUpdate(const sensor_msgs::Imu::ConstPtr& message);
  void odometryTimestampUpdate(const nav_msgs::Odometry::ConstPtr & message);
  void sonarLeftTimestampUpdate(const sensor_msgs::Range::ConstPtr& message);
  void sonarCenterTimestampUpdate(const sensor_msgs::Range::ConstPtr& message);
  void sonarRightTimestampUpdate(const sensor_msgs::Range::ConstPtr& message);
  
  // This function sends an array of floats
  // corresponding to predefined diagnostic values 
  // to be displayed in the GUI
  // For example, the wireless signal quality.
  void publishDiagnosticData();

  void simWorldStatsEventHandler(ConstWorldStatisticsPtr &msg);
  
  std::string getHumanFriendlyTime();
  
private:

  // These functions are called on a timer and check for problems with the sensors
  void sensorCheckTimerEventHandler(const ros::TimerEvent&);
  void simCheckTimerEventHandler(const ros::TimerEvent&);
  

  // Get the rate the simulation is running for simulated rovers
  float checkSimRate();
  
  void checkIMU();
  void checkGPS();
  void checkSonar();
  void checkCamera();
  void checkGripper();
  void checkOdometry();
    
  bool checkGPSExists();
  bool checkCameraExists();


  // This function checks the rover published name against the simulated rover model files.
  // If the name of this rover does not appear in the models then assume we are a
  // simulated rover. Being a simulated rover means that certain diagnostic checks will
  // be bypassed.
  bool checkIfSimulatedRover();
  
  // Takes the vendor and device IDs and searches the USB busses for a match
  bool checkUSBDeviceExists(uint16_t, uint16_t);
  
  ros::NodeHandle nodeHandle;
  ros::Publisher diagLogPublisher;
  ros::Publisher diagnosticDataPublisher;
  std::string publishedName;

  ros::Subscriber fingerAngleSubscribe;
  ros::Subscriber wristAngleSubscribe;
  ros::Subscriber imuSubscribe;
  ros::Subscriber odometrySubscribe;
  ros::Subscriber sonarLeftSubscribe;
  ros::Subscriber sonarCenterSubscribe;
  ros::Subscriber sonarRightSubscribe;
  
  float sensorCheckInterval = 2; // Check sensors every 2 seconds
  ros::Timer sensorCheckTimer;
  ros::Timer simCheckTimer;

  // Store some state about the current health of the rover
  bool cameraConnected = true;
  bool GPSConnected = true;
  bool simulated = false;
  bool fingersConnected;
  bool wristConnected;
  bool imuConnected;
  bool odometryConnected;
  bool sonarLeftConnected;
  bool sonarCenterConnected;
  bool sonarRightConnected;
  ros::Time fingersTimestamp;
  ros::Time wristTimestamp;
  ros::Time imuTimestamp;
  ros::Time odometryTimestamp;
  ros::Time sonarLeftTimestamp;
  ros::Time sonarCenterTimestamp;
  ros::Time sonarRightTimestamp;

  // Simulation update rate as a fraction of real time
  float simRate;
  gazebo::common::Time prevSimTime;
  gazebo::common::Time prevRealTime;
  
  
  WirelessDiags wirelessDiags;

  // So we can get Gazebo world stats
  gazebo::transport::NodePtr gazeboNode;
  gazebo::transport::SubscriberPtr worldStatsSubscriber;
};

#endif // End Diagnostics_h
