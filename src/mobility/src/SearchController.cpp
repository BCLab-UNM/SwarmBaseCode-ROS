#include "SearchController.h"
#include "StandardVars.h"
SearchController::SearchController() {
  rng = new random_numbers::RandomNumberGenerator();
}

/**
 * This code implements a basic random walk search.
 */
Result SearchController::search() {
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

void SearchController::setCurrentLocation(geometry_msgs::Pose2D currentLocation){
    
   this->currentLocation = currentLocation;
}

/**
 * Continues search pattern after interruption. For example, avoiding the
 * center or collisions.
 */
Result SearchController::continueInterruptedSearch(geometry_msgs::Pose2D oldSearchLocation){
    res.type = waypoint;
    
    geometry_msgs::Pose2D newSearchLocation;

  //remainingGoalDist avoids magic numbers by calculating the dist

  double remainingGoalDist = hypot(oldSearchLocation.x - currentLocation.x, oldSearchLocation.y - currentLocation.y);

  //this of course assumes random walk continuation. Change for diffrent search methods.
  newSearchLocation.theta = oldSearchLocation.theta;
  newSearchLocation.x = currentLocation.x + (0.50 * cos(oldSearchLocation.theta)); //(remainingGoalDist * cos(oldGoalLocation.theta));
  newSearchLocation.y = currentLocation.y + (0.50 * sin(oldSearchLocation.theta)); //(remainingGoalDist * sin(oldGoalLocation.theta));
  res.wpts.waypoints[0] = newSearchLocation;

  return res;
}
