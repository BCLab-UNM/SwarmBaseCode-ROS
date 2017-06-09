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
    Result CalculateResult();

    bool ShouldInterrept();
    void UpdateData(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D centerLocation);

   // sets the value of the current location 


  private:

    random_numbers::RandomNumberGenerator* rng;
    geometry_msgs::Pose2D currentLocation;
    geometry_msgs::Pose2D centerLocation;
    //struct for returning data to mobility
    Result result;
};

#endif /* SEARCH_CONTROLLER */
