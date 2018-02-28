#ifndef SEARCH_CONTROLLER
#define SEARCH_CONTROLLER

#include <random_numbers/random_numbers.h>
#include "Controller.h"

/**
 * This class implements the search control algorithm for the rovers. The code
 * here should be modified and enhanced to improve search performance.
 */
class SearchController : virtual Controller {

public:

  SearchController();

  void Reset() override;

  // performs search pattern
  Result DoWork() override;
  bool ShouldInterrupt() override;
  bool HasWork() override;

  // sets the value of the current location
  //void UpdateData(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D centerLocation);
  void SetCurrentLocation(Point currentLocation);
  void SetCenterLocation(Point centerLocation);
  void SetSuccesfullPickup();
  void SetDistance(float f) { distance = f; }

  bool GetTargetFound(){ targetFound; }
  bool SetTargetFound(bool val){ targetFound = val; }
  void SetObstSeen(bool b) { obstSeen = b; }
  int obstacleCalls = 0;

protected:

  void ProcessData();

private:

  random_numbers::RandomNumberGenerator* rng;
  Point currentLocation;
  Point centerLocation;
  Point clusterLocation;
  int attemptCount = 0;
  //struct for returning data to ROS adapter
  Result result;

  float initialHeading = 0;
  // Search state
  // Flag to allow special behaviour for the first waypoint
  bool first_waypoint = true;
  bool succesfullPickup = false;
  bool targetFound;
  float distance = 0.0;
  bool obstSeen = false;
};

#endif /* SEARCH_CONTROLLER */
