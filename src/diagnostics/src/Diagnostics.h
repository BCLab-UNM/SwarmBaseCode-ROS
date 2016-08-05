#ifndef Diagnostics_h
#define Diagnostics_h

#include <ros/ros.h>

// The following multiarray headers are for the diagnostics data publisher
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"

#include <string>
#include <exception>

//struct to hold collected information
struct SignalInfo {
  char mac[18];
  char ssid[33];
  int bandwidthAvailable;
  int level;
  int quality;
  int noise;
  float bandwidthUsed;
};

class Diagnostics {
  
public:
  Diagnostics(std::string);
  ~Diagnostics();
  void publishWarningLogMessage(std::string);
  void publishErrorLogMessage(std::string);
  void publishInfoLogMessage(std::string);
  
  // This function sends an array of floats
  // corresponding to predefined diagnostic values 
  // to be displayed in the GUI
  // For example, the wireless signal quality.
  void publishDiagnosticData();

  std::string getHumanFriendlyTime();
  
private:

  // These functions are called on a timer and check for problems with the sensors
  void sensorCheckTimerEventHandler(const ros::TimerEvent&);
  void checkIMU();
  void checkGPS();
  void checkSonar();
  void checkCamera();
  void checkWireless();
  
  bool checkGPSExists();
  bool checkCameraExists();


  // This function checks the rover published name against the simulated rover model files.
  // If the name of this rover does not appear in the models then assume we are a
  // simulated rover. Being a simulated rover means that certain diagnostic checks will
  // be bypassed.
  bool checkIfSimulatedRover();
  
  // Get wireless info
  SignalInfo getSignalInfo(const char *iwname);
  float calcBandwidthUsed();

  // Takes the vendor and device IDs and searches the USB busses for a match
  bool checkUSBDeviceExists(uint16_t, uint16_t);
  
  ros::NodeHandle nodeHandle;
  ros::Publisher diagLogPublisher;
  ros::Publisher diagnosticDataPublisher;
  std::string publishedName;

  
  float sensorCheckInterval = 2; // Check sensors every 2 seconds
  ros::Timer sensorCheckTimer;

  // Store some state about the current health of the rover
  bool cameraConnected = true;
  bool GPSConnected = true;
  bool simulated = false;

  // State for the bandwith calculation
  long int prev_total_bytes = 0;
  long int total_bytes = 0;
};

#endif // End Diagnostics_h
