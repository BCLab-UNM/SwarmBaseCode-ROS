#include "Diagnostics.h"
#include <usb.h>

#include <std_msgs/String.h> // For creating ROS string messages
#include <ctime> // For time()

using namespace std;

Diagnostics::Diagnostics(std::string name) {
  this->publishedName = name;
  diagLogPublisher = nodeHandle.advertise<std_msgs::String>("/diagsLog", 1, true);

  // Setup sensor check timers
  sensorCheckTimer = nodeHandle.createTimer(ros::Duration(sensorCheckInterval), &Diagnostics::sensorCheckTimerEventHandler, this);
  publishInfoLogMessage("Diagnostics Package Started");
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
void Diagnostics::sensorCheckTimerEventHandler(const ros::TimerEvent& event)
{
  checkIMU();
  checkGPS();
  checkSonar();
  checkCamera();
}

void Diagnostics::checkIMU() {
  // Example
  //publishWarningLogMessage("IMU Warning");
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
    publishErrorLogMessage("GPS not connected");
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
    publishErrorLogMessage("Camera not connected");
    cameraConnected = false;
  }
}

void Diagnostics::checkSonar() {
  //Example
  //publishErrorLogMessage("Sonar Error");
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

Diagnostics::~Diagnostics() {
}
