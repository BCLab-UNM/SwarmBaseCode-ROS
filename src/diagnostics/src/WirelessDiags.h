#ifndef WirelessDiags_h
#define WirelessDiags_h

#include <string> 
#include <sys/time.h>

// Wifi device status data structure
struct WirelessInfo {
  char mac[18];
  char ssid[33];
  int bandwidthAvailable;
  int level;
  int quality;
  int noise;
  float bandwidthUsed;
};

class WirelessDiags {

public:

  WirelessDiags();
  
  // Get and set wifi device network id
  std::string setInterface();

  // Get wifi device status information
  WirelessInfo getInfo();
  
private:

  // Determine if wifi device is enabled and active
  bool isInterfaceUp(std::string name);

  // Find the first wifi device listed in ubuntu system files
  std::string findWirelessInterface();

  // Is network interface type wireless
  bool isWireless(const char* name);

  // Calculate bitrate using the number of bytes sent and received
  float calcBitRate();

  // Network id
  std::string interfaceName;

  // Times and bytes sent/recieved (used to calculate bit rate)
  long int prev_total_bytes = 0;
  long int total_bytes = 0;
  struct timeval prev_time;
};

#endif // WirelessDiags_h
