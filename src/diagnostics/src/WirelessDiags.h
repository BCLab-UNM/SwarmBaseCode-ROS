#ifndef WirelessDiags_h
#define WirelessDiags_h

#include <string> // wireless device interface name
#include <sys/time.h> // gettimeofday and timeval

//struct to hold collected information
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
  
  // Sets the name of the interface
  // about which to provide information
  // returns the name of the wireless interface
  std::string setInterface();

  WirelessInfo getInfo();
  
private:


  // We don't want to try and get info about a network interface that doesn't exist
  bool isInterfaceUp(std::string name);

  std::string findWirelessInterface();

  // checks if a network interface is wireless or not
  bool isWireless(const char* name);
  float calcBitRate();

  std::string interfaceName;

  // State needed to keep track of
  // the number of bytes sent between calcBitRate calls

  long int prev_total_bytes = 0;
  long int total_bytes = 0;
  struct timeval prev_time; // The wall time of the last call to calcBitRate;
};

#endif // WirelessDiags_h
