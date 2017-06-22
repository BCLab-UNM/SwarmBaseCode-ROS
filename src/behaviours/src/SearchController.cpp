#include "SearchController.h"

SearchController::SearchController() {
  rng = new random_numbers::RandomNumberGenerator();
  currentLocation.x = 0;
  currentLocation.y = 0;
  currentLocation.theta = 0;

  centerLocation.x = 0;
  centerLocation.y = 0;
  centerLocation.theta = 0;
  result.PIDMode = FAST_PID;
}

void SearchController::Reset() {
    result.reset = false;
}

/**
 * This code implements a basic random walk search.
 */
Result SearchController::DoWork() {
   result.type = waypoint;
   Point  searchLocation;
      
  //select new heading from Gaussian distribution around current heading
  searchLocation.theta = rng->gaussian(currentLocation.theta, 0.25);
  
  //selec.t new position 50 cm from current location
  searchLocation.x = currentLocation.x + (0.5 * cos(searchLocation.theta));
  searchLocation.y = currentLocation.y + (0.5 * sin(searchLocation.theta));

  result.wpts.waypoints.clear();
  result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);
         
  return result;
  
}

void SearchController::setCenterLocation(Point centerLocation) {
    this->centerLocation = centerLocation;
}

void SearchController::setCurrentLocation(Point currentLocation) {
    this->currentLocation = currentLocation;
}

void SearchController::ProcessData() {

}

bool SearchController::ShouldInterrupt(){
    ProcessData();

    return nullptr;
}

bool SearchController::HasWork() {
    return true;
}




