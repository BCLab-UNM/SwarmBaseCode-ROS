#ifndef Diagnostics_h
#define Diagnostics_h

#include <ros/ros.h>
#include <string>

class Diagnostics {
  
public:
  Diagnostics(std::string);
  ~Diagnostics();
  void publishWarningLogMessage(std::string);
  void publishErrorLogMessage(std::string);
  void publishInfoLogMessage(std::string);
  std::string getHumanFriendlyTime();
  
private:

  // These functions are called on a timer and check for problems with the sensors
  void sensorCheckTimerEventHandler(const ros::TimerEvent&);
  void checkIMU();
  void checkGPS();
  void checkSonar();

  bool checkGPSExists();
  
  ros::NodeHandle nodeHandle;
  ros::Publisher diagLogPublisher;
  std::string publishedName;

  float sensorCheckInterval = 100; // Check sensors every 10 seconds
  ros::Timer sensorCheckTimer;
  
};

#endif // End Diagnostics_h
