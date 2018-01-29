#ifndef Diagnostics_h
#define Diagnostics_h

#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <ros/ros.h>

#include <std_msgs/String.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/NavSatFix.h>

#include "WirelessDiags.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"

#include <string>
#include <exception>

class Diagnostics {
  
public:
  Diagnostics(std::string);
  ~Diagnostics();

  // Publish rover status error/info. Displayed in tabs (bottom left of the gui)
  void publishWarningLogMessage(std::string); // NOTE: NOT USED
  void publishErrorLogMessage(std::string);
  void publishInfoLogMessage(std::string);

  // Subscriber handlers that record timestamp of last successfully published message
  void fingerTimestampUpdate(const geometry_msgs::QuaternionStamped::ConstPtr& message);
  void wristTimestampUpdate(const geometry_msgs::QuaternionStamped::ConstPtr& message);
  void imuTimestampUpdate(const sensor_msgs::Imu::ConstPtr& message);
  void odometryTimestampUpdate(const nav_msgs::Odometry::ConstPtr & message);
  void sonarLeftTimestampUpdate(const sensor_msgs::Range::ConstPtr& message);
  void sonarCenterTimestampUpdate(const sensor_msgs::Range::ConstPtr& message);
  void sonarRightTimestampUpdate(const sensor_msgs::Range::ConstPtr& message);
  void abridgeNode(std_msgs::String msg);
  void sbridgeNode(std_msgs::String msg);
  void behaviourNode(std_msgs::String msg);
  void ubloxNode(const sensor_msgs::NavSatFixConstPtr& message);
  
  // Publish wifi signal quality (!simulation). Displayed next to rover name (top right gui)
  void publishDiagnosticData();

  // Subscriber handler that collects simulation times to determine simulation rate.
  void simWorldStatsEventHandler(ConstWorldStatisticsPtr &msg);
  
  // Return the current time in this timezone in "WeekDay Month Day hr:mni:sec year" format (thread-safe)
  std::string getHumanFriendlyTime();
  
private:

  // Timer event handler that publishes diagnostics every sensorCheckInterval (IMU, GPS, sonar, camera, gripper, odom)
  void sensorCheckTimerEventHandler(const ros::TimerEvent&);
  
  // Timer event handler that publishes diagnostics every sensorCheckInterval (simulation times)
  void simCheckTimerEventHandler(const ros::TimerEvent&);
  
  // Timer event handler that publishes diagnostics every sensorCheckInterval (abridge/sbridge, ublox)
  void nodeCheckTimerEventHandler(const ros::TimerEvent&);
  
  // Timer event handler method helpers 
  float checkSimRate();
  void checkIMU();
  void checkGPS();
  void checkSonar();
  void checkCamera();
  void checkGripper();
  void checkOdometry();
  void checkAbridge();
  void checkSbridge();
  void checkBehaviour();
  void checkUblox();

  // Check that a U-Blox device exists in the connected USB devices list
  bool checkGPSExists();

  // Check that a Logitec c170 device exists in the connected USB devices list
  bool checkCameraExists();


  // This function checks the rover published name against the simulated rover model files.
  // If the name of this rover does not appear in the models then assume we are a
  // simulated rover. Being a simulated rover means that certain diagnostic checks will
  // be bypassed. (a temporary hack)
  bool checkIfSimulatedRover();
  
  // Search connected USB devices for a matching device
  bool checkUSBDeviceExists(uint16_t, uint16_t);
  
  // Accessor to roscpp's inteface for creating subscribers, publishers
  ros::NodeHandle nodeHandle;
  
  // Diagnostics publisher - rover status error/info
  ros::Publisher diagLogPublisher;

  // Diagnostics publisher - wireless quality or simulation rate
  ros::Publisher diagnosticDataPublisher;

  std::string publishedName;

  // Subscribers to sensors, gripper, odom, abridge/sbridge published messages
  ros::Subscriber fingerAngleSubscribe;
  ros::Subscriber wristAngleSubscribe;
  ros::Subscriber imuSubscribe;
  ros::Subscriber odometrySubscribe;
  ros::Subscriber sonarLeftSubscribe;
  ros::Subscriber sonarCenterSubscribe;
  ros::Subscriber sonarRightSubscribe;
  ros::Subscriber abdridgeNodeSubscribe;
  ros::Subscriber sbdridgeNodeSubscribe;
  ros::Subscriber behaviourNodeSubscribe;
  ros::Subscriber ubloxNodeSubscribe;
  
  float sensorCheckInterval = 2; // Check sensors every 2 seconds
  float nodeCheckInterval = 5; //Check nodes every 5 seconds
  ros::Timer sensorCheckTimer;
  ros::Timer simCheckTimer;
  ros::Timer nodeCheckTimer;

  ros::Time diagnostics_start_time; // Time that this package started
  float node_start_delay; // Time to wait for nodes to start

  // Initialise values according to whether the rover will report the
  // connection on startup or only after recovery from a failure.
  bool cameraConnected = true;
  bool GPSConnected = true;
  bool simulated = false;
  bool fingersConnected = false;
  bool wristConnected = false;
  bool imuConnected = false;
  bool odometryConnected = false;
  bool sonarLeftConnected = false;
  bool sonarCenterConnected = false;
  bool sonarRightConnected = false;
  bool abridgeRunning = true;
  bool sbridgeRunning = true;
  bool behaviourRunning = true;
  bool ubloxRunning = true;

  // Record time stamp of the last message received
  ros::Time fingersTimestamp;
  ros::Time wristTimestamp;
  ros::Time imuTimestamp;
  ros::Time odometryTimestamp;
  ros::Time sonarLeftTimestamp;
  ros::Time sonarCenterTimestamp;
  ros::Time sonarRightTimestamp;
  ros::Time abridgeNodeTimestamp;
  ros::Time sbridgeNodeTimestamp;
  ros::Time behaviourNodeTimestamp;
  ros::Time ubloxNodeTimestamp;

  // Max time since last heartbeat before notifying the user - in seconds
  float node_heartbeat_timeout, device_heartbeat_timeout;

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
