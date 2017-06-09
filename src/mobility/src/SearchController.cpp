#include "SearchController.h"
#include "StandardVars.h"
SearchController::SearchController() {
  rng = new random_numbers::RandomNumberGenerator();
  currentLocation.x = 0;
  currentLocation.y = 0;
  currentLocation.theta = 0;

  centerLocation.x = 0;
  centerLocation.y = 0;
  centerLocation.theta = 0;
}

/**
 * This code implements a basic random walk search.
 */
Result SearchController::CalculateResult() {
   result.type = waypoint;
   geometry_msgs::Pose2D  searchLocation;
      
  //select new heading from Gaussian distribution around current heading
  searchLocation.theta = rng->gaussian(currentLocation.theta, 0.25);
  
  //selec.t new position 50 cm from current location
  searchLocation.x = currentLocation.x + (0.5 * cos(searchLocation.theta));
  searchLocation.y = currentLocation.y + (0.5 * sin(searchLocation.theta));

  result.wpts.waypoints.clear();
  result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);
         
  return result;
  
}
 void SearchController::UpdateData(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D centerLocation){
     this->currentLocation = currentLocation;
     this->centerLocation = centerLocation;
}

bool SearchController::ShouldInterrept(){
    return nullptr;
}





