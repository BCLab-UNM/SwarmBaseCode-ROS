#ifndef SEARCH_CONTROLLER
#define SEARCH_CONTROLLER

#include <geometry_msgs/Pose2D.h>
#include <random_numbers/random_numbers.h>
#include "StandardVars.h"
/**
 * This class implements the search control algorithm for the rovers. The code
 * here should be modified and enhanced to improve search performance.
 */
class SearchController {

  public:

    SearchController();

    // performs search pattern
    Result search();

    // continues search pattern after interruption
   Result continueInterruptedSearch(geometry_msgs::Pose2D oldGoalLocation);
   // sets the value of the current location 


  private:

    void setCurrentLocation(geometry_msgs::Pose2D setLocation);
    random_numbers::RandomNumberGenerator* rng;
    geometry_msgs::Pose2D currentLocation;
    //struct for returning data to mobility
    Result res;
};

#endif /* SEARCH_CONTROLLER */
