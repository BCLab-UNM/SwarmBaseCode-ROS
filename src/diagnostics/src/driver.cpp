// Application entry point for Diagnostics node:
//   -instantiates the diagnostics classes
//   -ROS initializations
//   -process ROS events
//   -determine the name to publish under from command line argument or hostname

#include "Diagnostics.h"
#include <string>
#include <signal.h>
#include <gazebo/gazebo.hh>

using namespace std;

void sigintEventHandler(int signal); // Ubuntu signal handler

int main(int argc, char** argv) {

  sleep(10);
  char host[128];
  gethostname(host, sizeof (host));
  std::string hostname(host);
  
  std::string publishedName;
  if (argc >= 2) {
    publishedName = argv[1];
    cout << "Welcome to the world of tomorrow " << publishedName << "!  Diagnostic module started." << endl;
  } else {
    publishedName = hostname;
    cout << "No Name Selected. Default is: " << publishedName << endl;
  }
  
  // Declare no signal handler so we can catch SIGINT ourselves and shutdown the node
  ros::init(argc, argv, (publishedName + "_DIAGNOSTICS"), ros::init_options::NoSigintHandler);
    
  Diagnostics diags(publishedName);
  
  signal(SIGINT, sigintEventHandler); // Register the SIGINT event handler so the node can shutdown properly
  
  ros::spin(); // Process ROS events
  
  return EXIT_SUCCESS;
}

void sigintEventHandler(int sig) {
  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}

