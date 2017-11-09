#include "WirelessDiags.h"

#include <linux/types.h>
#include <sys/socket.h> // For sockets
#include <fcntl.h> // For socket open
#include <unistd.h> // For socket close
#include <stdexcept> // For runtime_error
#include <linux/wireless.h> // For wifi stats 
#include <sys/ioctl.h> // For communication with the kernel
#include <iostream> // For reading system info
#include <fstream> // "
#include <cstring> // For memset
#include <arpa/inet.h> // For IPPROTO_IP
#include <ifaddrs.h> // For network interface struct

using namespace std;

WirelessDiags::WirelessDiags() {
  prev_total_bytes = 0;
  gettimeofday(&prev_time,NULL); // Set the prevtime to be the time this object was created using the default time zone

}

// Sets the diagnostics to use the first wireless interfact found
string WirelessDiags::setInterface() {

  string name = findWirelessInterface();

   if (!isInterfaceUp(name)) throw runtime_error("No such network interface: " + name);

  interfaceName = name;

  calcBitRate(); // Initialize the previous byte counts

  return name;
}


// Check if the interface we were told to use exists
bool WirelessDiags::isInterfaceUp(string name) {
    struct ifreq ifr;
    int sock = socket(PF_INET6, SOCK_DGRAM, IPPROTO_IP);
    memset(&ifr, 0, sizeof(ifr));
    strcpy(ifr.ifr_name, name.c_str());
    if (ioctl(sock, SIOCGIFFLAGS, &ifr) < 0) {
      string errorMsg = "WirelessDiags::isInterfaceUp(): error getting network interface flags. Unable to open ioctl socket for " + name + ": "+ string(strerror(errno));
      throw runtime_error(errorMsg);
    }
    close(sock);
    return !!(ifr.ifr_flags & IFF_UP);
}

string WirelessDiags::findWirelessInterface() {
  
  string wirelessName = "none";
    struct ifaddrs *ifaddr, *ifa;
    
    // Populate the struct
    if (getifaddrs(&ifaddr) == -1) {
	perror("getifaddrs");
	return "fail";
      }

    // Iterate over the network interfaces and return the first one that is wireless
    for (ifa = ifaddr; ifa; ifa = ifa->ifa_next) {
      
      // Interface is null or not a network interface
      if (!ifa->ifa_addr || ifa->ifa_addr->sa_family != AF_PACKET) continue;
      
      // Check if the interface is wireless and return it's name if so
      if (isWireless(ifa->ifa_name)) {
	wirelessName = string(ifa->ifa_name);
	break;
      } 
    }
    
    // Free the struct containing interface information
    freeifaddrs(ifaddr);

    return wirelessName;
}

// Code to check whether a network interface is wireless or not.
bool WirelessDiags::isWireless(const char* name) {
  int sock = -1;
  struct iwreq pwrq;
  memset(&pwrq, 0, sizeof(pwrq));
  strncpy(pwrq.ifr_name, name, IFNAMSIZ);

  // Try to open a socket to the interface
  sock = socket(AF_INET, SOCK_STREAM, 0);

  bool wireless = false;
  
  // Check if wireless by asking for verfification
  // of wireless extensions (the SIOCGIWNAME directive)
  if (ioctl(sock, SIOCGIWNAME, &pwrq) != -1) wireless =  true;

  close(sock);
  return wireless;
}

// Helper function to read the number of bytes sent and received over time to calculate
// the current bitrate
float WirelessDiags::calcBitRate() {

  // Path to the linux provided stats. These files are pointers to memory locations
  // and are not on disk.
  string receive_bytes_stat_path = "/sys/class/net/"+interfaceName+"/statistics/rx_bytes";
  string transmit_bytes_stat_path = "/sys/class/net/"+interfaceName+"/statistics/tx_bytes";

  // Open, read and close the bytes received and bytes sent file pointers. 
  ifstream receive_bytes_stat, transmit_bytes_stat;

  // Throw an exception if there was a problem
  receive_bytes_stat.exceptions( std::ifstream::failbit | std::ifstream::badbit );
  transmit_bytes_stat.exceptions( std::ifstream::failbit | std::ifstream::badbit );
  
  // Open the files
  transmit_bytes_stat.open(transmit_bytes_stat_path.c_str());
  receive_bytes_stat.open(receive_bytes_stat_path.c_str());
  
  string receive_stat;
  getline(receive_bytes_stat,receive_stat);
  receive_bytes_stat.close();
   
  string transmit_stat;
  getline(transmit_bytes_stat,transmit_stat);
  transmit_bytes_stat.close();

  // Remember the total bytes transmitted so we can take the difference 
  // between recordings at each time interval.
  prev_total_bytes = total_bytes;
   
  // Convert string to int
  string::size_type sz;   // alias of size_t
  long int rx_bytes = 0;
  long int tx_bytes = 0;
  if (!receive_stat.empty())
  {
    rx_bytes = stoi(receive_stat,&sz);
  }
  if (!transmit_stat.empty())
  {
    tx_bytes = stoi(transmit_stat,&sz);
  }

  // Get the total bytes transmitted and recevied. This is the total bandwidth used.
  total_bytes = rx_bytes + tx_bytes;

  // If we haven't set the value of the previous reading skip. This will never be zero since it is the number of
  // bytes sent and received since boot. 
  if (prev_total_bytes == 0) return 0;
   
  // Get the bytes transmitted and received as a function of time.
  // This assumes the function is called using the sensor check function.
  // Perhaps this function should keep track of the time elapsed itself?

  struct timeval now;
  gettimeofday(&now, NULL);
        
  // compute the elapsed time in seconds with millisecond resolution
  double elapsedTime = (now.tv_sec - prev_time.tv_sec) * 1000.0;      // sec to ms
  elapsedTime += (now.tv_usec - prev_time.tv_usec) / 1000.0;   // us to ms

  // Now convert milliseconds to seconds
  elapsedTime /= 1000;

  // Remember when this was called so we can calculate the delay next
  // time it is called
  prev_time = now; 
  
  float byte_rate = byte_rate = (total_bytes - prev_total_bytes)*1.0f/elapsedTime;
   
  // Rate in B/s
  return byte_rate;
}

WirelessInfo WirelessDiags::getInfo(){
  
  WirelessInfo sigInfo;

  // Declare an iwreq (wireless interface request object) for use in IOCTL communication
  iwreq req;

  // Allocate space for the request object
  memset(&req, 0, sizeof(struct iwreq));

  // Populate the interface name in the request object
  strcpy(req.ifr_name, interfaceName.c_str());

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
    string errorMsg = "WirelessDiags::getInfo(): error getting wireless statistics. Unable to open ioctl socket for " + interfaceName + ": "+ string(strerror(errno));

    // Throw an error
    throw runtime_error(errorMsg);
    return sigInfo;
  }
  else if(((iw_statistics *)req.u.data.pointer)->qual.updated & IW_QUAL_DBM){
    // Opened the socket so read the data
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
    // There was an error throw an exception
    string errorMsg = "WirelessDiags::getInfo(): error getting network ESSID. Unable to open ioctl socket for " + interfaceName + ": "+ string(strerror(errno));
    throw runtime_error(errorMsg);
  }
  else {
    // Opened the socket so read the data
    memcpy(&sigInfo.ssid, req.u.essid.pointer, req.u.essid.length);
    memset(&sigInfo.ssid[req.u.essid.length],0,1);
  }

  //SIOCGIWRATE for bits/sec (convert to mbit)
  int bitrate=-1;
  
  //this will get the claimed bitrate of the link
  if(ioctl(sockfd, SIOCGIWRATE, &req) == -1){
    // There was an error throw an exception
    string errorMsg = "WirelessDiags::getInfo(): error getting transmission rate. Unable to open ioctl socket for " + interfaceName + ": "+ string(strerror(errno));
    throw runtime_error(errorMsg);
  } else {
    // Opened the socket so read the data
    memcpy(&bitrate, &req.u.bitrate, sizeof(int));
    sigInfo.bandwidthAvailable=bitrate/1000000;
  }

  //SIOCGIFHWADDR for mac addr
  ifreq req2;
  strcpy(req2.ifr_name, interfaceName.c_str());

  //this will get the mac address of the interface
  if(ioctl(sockfd, SIOCGIFHWADDR, &req2) == -1){
    // There was an error throw an exception
    string errorMsg = "WirelessDiags::getInfo(): error getting MAC address. Unable to open ioctl socket for " + interfaceName + ": "+ string(strerror(errno));
    throw runtime_error(errorMsg);
  } else{
    // Opened the socket so read the data
    sprintf(sigInfo.mac, "%.2X", (unsigned char)req2.ifr_hwaddr.sa_data[0]);
    for(int s=1; s<6; s++){
      sprintf(sigInfo.mac+strlen(sigInfo.mac), ":%.2X", (unsigned char)req2.ifr_hwaddr.sa_data[s]);
    }
  }

  // We are done so clean up
  close(sockfd);

  sigInfo.bandwidthUsed = calcBitRate();  

  return sigInfo;
}
