#ifndef SEARCH_CONTROLLER
#define SEARCH_CONTROLLER

#include <geometry_msgs/Pose2D.h>
#include <random_numbers/random_numbers.h>
#include "Controller.h"
#include "StandardVars.h"
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
    void UpdateData(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D centerLocation);

protected:

    void ProcessData();

private:

    random_numbers::RandomNumberGenerator* rng;
    geometry_msgs::Pose2D currentLocation;
    geometry_msgs::Pose2D centerLocation;
    //struct for returning data to mobility
    Result result;
};

#endif /* SEARCH_CONTROLLER */
