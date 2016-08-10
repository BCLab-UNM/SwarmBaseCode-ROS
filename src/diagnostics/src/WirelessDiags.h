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

  // The constructor takes the name of the interface
  // about which to provide information
  WirelessDiags(std::string);

  WirelessInfo getInfo();

private:

  float calcBitRate();

  std::string interfaceName;

  // State needed to keep track of
  // the number of bytes sent between calcBitRate calls

  long int prev_total_bytes = 0;
  long int total_bytes = 0;
  struct timeval prev_time; // The wall time of the last call to calcBitRate;
};

#endif // WirelessDiags_h
