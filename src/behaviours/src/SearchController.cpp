#include "SearchController.h"
#include <angles/angles.h>

SearchController::SearchController()
{
  rng = new random_numbers::RandomNumberGenerator();
  currentLocation.x = 0;
  currentLocation.y = 0;
  currentLocation.theta = 0;

  centerLocation.x = 0;
  centerLocation.y = 0;
  centerLocation.theta = 0;
  result.PIDMode = FAST_PID;

  result.fingerAngle = M_PI/2;
  result.wristAngle = M_PI/4;
  distance = 1;
}

void SearchController::Reset() {
  result.reset = false;
}

/**
 * This code implements a basic random walk search.
 */
Result SearchController::DoWork()
{
  result.type = waypoint;
  Point  searchLocation;

  if(!seenTags)
  {
    distance++;
  }

  searchLocation.x = 0;
  searchLocation.y = 0;

  searchLocation.x = centerLocation.x + ((distance * 0.5) * cos(searchLocation.theta));
  searchLocation.y = centerLocation.y + ((distance * 0.5) * sin(searchLocation.theta));

  cout << "driving forward .5 meters" << endl;

  if(distance >= 10)
  {
    searchLocation = centerLocation;
    distance = 1;
  }

  /*
  //select new position 50 cm from current location
  if (first_waypoint)
  {
    first_waypoint = false;
    searchLocation.theta = currentLocation.theta + M_PI;
    searchLocation.x = currentLocation.x + (0.5 * cos(searchLocation.theta));
    searchLocation.y = currentLocation.y + (0.5 * sin(searchLocation.theta));
  }
  else
  {
    //select new heading from Gaussian distribution around current heading
    searchLocation.theta = rng->gaussian(currentLocation.theta, 0.785398); //45 degrees in radians
    searchLocation.x = currentLocation.x + (0.5 * cos(searchLocation.theta));
    searchLocation.y = currentLocation.y + (0.5 * sin(searchLocation.theta));
  }
*/
  result.wpts.waypoints.clear();
  result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);

  return result;
}

void SearchController::SetCenterLocation(Point centerLocation) {
  
  float diffX = this->centerLocation.x - centerLocation.x;
  float diffY = this->centerLocation.y - centerLocation.y;
  this->centerLocation = centerLocation;
  
  if (!result.wpts.waypoints.empty())
  {
  result.wpts.waypoints.back().x -= diffX;
  result.wpts.waypoints.back().y -= diffY;
  }
  
}

void SearchController::SetCurrentLocation(Point currentLocation) {
  this->currentLocation = currentLocation;
}

void SearchController::ProcessData() {
}

bool SearchController::ShouldInterrupt(){
  ProcessData();

  return false;
}

bool SearchController::HasWork() {
  return true;
}

void SearchController::SetSuccesfullPickup() {
  succesfullPickup = true;
}


