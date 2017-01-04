#include "SearchController.h"

SearchController::SearchController() {
  rng = new random_numbers::RandomNumberGenerator();
}

/**
 * This code implements a basic random walk search.
 */
geometry_msgs::Pose2D SearchController::search(geometry_msgs::Pose2D currentLocation) {
  geometry_msgs::Pose2D goalLocation;

  //select new heading from Gaussian distribution around current heading
  goalLocation.theta = rng->gaussian(currentLocation.theta, 0.25);

  //select new position 50 cm from current location
  goalLocation.x = currentLocation.x + (0.5 * cos(goalLocation.theta));
  goalLocation.y = currentLocation.y + (0.5 * sin(goalLocation.theta));

  return goalLocation;
}

/**
 * If you want to avoid the center diffrently place code here.
 * This code keeps the robot from driving over the center when searching for blocks.
 */
geometry_msgs::Pose2D SearchController::continueInterruptedSearch(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D oldGoalLocation) {
  geometry_msgs::Pose2D newGoalLocation;

  //remainingGoalDist avoids magic numbers by calculating the dist
  double remainingGoalDist = hypot(oldGoalLocation.x - currentLocation.x, oldGoalLocation.y - currentLocation.y);

  //this of course assumes random walk continuation. Change for diffrent search methods.
  newGoalLocation.theta = oldGoalLocation.theta;
  newGoalLocation.x = currentLocation.x + (remainingGoalDist * cos(oldGoalLocation.theta));
  newGoalLocation.y = currentLocation.y + (remainingGoalDist * sin(oldGoalLocation.theta));

  return newGoalLocation;
}
