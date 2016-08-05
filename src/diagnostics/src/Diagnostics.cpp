#include "Diagnostics.h"

#include <linux/types.h>
#include <sys/socket.h>

#include <linux/wireless.h>
#include <sys/ioctl.h>

#include <string>

#include <usb.h>
#include <sys/stat.h>
#include <std_msgs/String.h> // For creating ROS string messages
#include <ctime> // For time()

using namespace std;

Diagnostics::Diagnostics(std::string name) {
  this->publishedName = name;
  diagLogPublisher = nodeHandle.advertise<std_msgs::String>("/diagsLog", 1, true);
  diagnosticDataPublisher  = nodeHandle.advertise<std_msgs::Float32MultiArray>("/"+publishedName+"/diagnostics", 10);

  // Setup sensor check timers
  sensorCheckTimer = nodeHandle.createTimer(ros::Duration(sensorCheckInterval), &Diagnostics::sensorCheckTimerEventHandler, this);
  if ( checkIfSimulatedRover() ) {
    simulated = true;
    publishInfoLogMessage("Diagnostic Package Started. Simulated Rover.");
  } else { 
    simulated = false;
    publishInfoLogMessage("Diagnostic Package Started. Physical Rover.");
  }
}

SignalInfo Diagnostics::getSignalInfo(const char *iwname){

  SignalInfo sigInfo;

  // Declare an iwreq (wireless interface request object) for use in IOCTL communication

iwreq req;

// Allocate space for the request object
memset(&req, 0, sizeof(struct iwreq));

// Populate the interface name in the request object
strcpy(req.ifr_name, iwname);

// Declare an iw_statistics object to store the IOCTL results in
iw_statistics *stats;

// Create socket through which to talk to the kernel 
int sockfd = socket(AF_INET, SOCK_DGRAM, 0);


// Allocate space for the iw_statistics object, point to it from the request,
// and store the length of the object in the request.
req.u.data.pointer = (iw_statistics *)malloc(sizeof(iw_statistics));
req.u.data.length = sizeof(iw_statistics);


// Use IOCTL to request the wireless stats. If -1 there was an error.
if(ioctl(sockfd, SIOCGIWSTATS, &req) == -1){

  string errorMsg = "Unable to open ioctl socket for " + string(iwname) + ": "+ string(strerror(errno));

  // Publish the error to the diagnostics log
  publishErrorLogMessage(errorMsg);
  return sigInfo;
}
else if(((iw_statistics *)req.u.data.pointer)->qual.updated & IW_QUAL_DBM){

//signal is measured in dBm and is valid for us to use

//sigInfo->level=((iw_statistics *)req.u.data.pointer)->qual.level - 256;
sigInfo.level = ((iw_statistics *)req.u.data.pointer)->qual.level - 256;
sigInfo.quality=((iw_statistics *)req.u.data.pointer)->qual.qual;
sigInfo.noise=((iw_statistics *)req.u.data.pointer)->qual.noise;

}

//SIOCGIWESSID for ssid

char buffer[32];

memset(buffer, 0, 32);

req.u.essid.pointer = buffer;

req.u.essid.length = 32;

//this will gather the SSID of the connected network

if(ioctl(sockfd, SIOCGIWESSID, &req) == -1){

//die with error, invalid interface

return sigInfo;

}

 else{

memcpy(&sigInfo.ssid, req.u.essid.pointer, req.u.essid.length);

memset(&sigInfo.ssid[req.u.essid.length],0,1);

}


//SIOCGIWRATE for bits/sec (convert to mbit)

int bitrate=-1;

//this will get the bitrate of the link

if(ioctl(sockfd, SIOCGIWRATE, &req) == -1){

fprintf(stderr, "bitratefail");

return sigInfo;

}else{

memcpy(&bitrate, &req.u.bitrate, sizeof(int));

sigInfo.bitrate=bitrate/1000000;

}



//SIOCGIFHWADDR for mac addr

ifreq req2;

strcpy(req2.ifr_name, iwname);

//this will get the mac address of the interface

if(ioctl(sockfd, SIOCGIFHWADDR, &req2) == -1){

fprintf(stderr, "mac error");

return sigInfo;

}

 else{

sprintf(sigInfo.mac, "%.2X", (unsigned char)req2.ifr_hwaddr.sa_data[0]);

for(int s=1; s<6; s++){

sprintf(sigInfo.mac+strlen(sigInfo.mac), ":%.2X", (unsigned char)req2.ifr_hwaddr.sa_data[s]);

}

}

close(sockfd);
  

 return sigInfo;
}

void Diagnostics::publishDiagnosticData() {
  string interfaceName = "wlan1";

  SignalInfo info = getSignalInfo(interfaceName.c_str());

  std_msgs::Float32MultiArray rosMsg;
  rosMsg.data.clear();
  rosMsg.data.push_back(info.quality);
  diagnosticDataPublisher.publish(rosMsg);
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
void Diagnostics::sensorCheckTimerEventHandler(const ros::TimerEvent& event) {

  publishInfoLogMessage("Scheduled diagnostic update occured.");

  if (!simulated) {
  checkIMU();
  checkGPS();
  checkSonar();
  checkCamera();
  checkWireless();

  publishDiagnosticData();
  }

}

void Diagnostics::checkWireless() {

  //  string interfaceName = "wlan1";
  //SignalInfo info = getSignalInfo(interfaceName.c_str());


  //publishInfoLogMessage("Wireless " + interfaceName + ":\n"
  //		       + "Connected to: " + string(info.ssid) + "\n"
  //		       + "Quality: " + to_string(info.quality) + "\n"
  //		       + "Level: " + to_string(info.level) + "\n"
  //		       + "Bitrate: " + to_string(info.bitrate) + "\n");

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
}

