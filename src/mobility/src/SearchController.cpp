#include "SearchController.h"
#include "StandardVars.h"
SearchController::SearchController() {
  rng = new random_numbers::RandomNumberGenerator();
}

/**
 * This code implements a basic random walk search.
 */
Result SearchController::CalculateResult() {
   res.type = waypoint;
   geometry_msgs::Pose2D  searchLocation;
      
  //select new heading from Gaussian distribution around current heading
  searchLocation.theta = rng->gaussian(currentLocation.theta, 0.25);
  
  //selec.t new position 50 cm from current location
  searchLocation.x = currentLocation.x + (0.5 * cos(searchLocation.theta));
  searchLocation.y = currentLocation.y + (0.5 * sin(searchLocation.theta));
  res.wpts.waypoints[0] = searchLocation;
         
  return res;
  
}
 void SearchController::UpdateData(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D centerLocation){
     this->currentLocation = currentLocation;
     this->centerLocation = currentLocation;


}

bool SearchController::ShouldInterrept(){
    return nullptr;
}





