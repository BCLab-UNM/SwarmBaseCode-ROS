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

    cout << "in search controller" << endl;

    if (!result.wpts.waypoints.empty()) {
        if (hypot(result.wpts.waypoints[0].x-currentLocation.x, result.wpts.waypoints[0].y-currentLocation.y) < 0.10) {
            attemptCount = 0;
            cout << "distance was less than tolerance new search" << endl;
        }
    }

    if (attemptCount > 0 && attemptCount < 5) {
        attemptCount++;
        cout <<"attempt again**********************************" << endl;
        return result;
    }
    else if (attemptCount >= 5 || attemptCount == 0) {
        cout << "new search point*****************************" << endl;
        attemptCount = 1;


        result.type = waypoint;
        Point  searchLocation;

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
	    searchLocation.theta = rng->gaussian(currentLocation.theta, 0.25);
	    searchLocation.x = currentLocation.x + (0.5 * cos(searchLocation.theta));
	    searchLocation.y = currentLocation.y + (0.5 * sin(searchLocation.theta));
	  }

	result.wpts.waypoints.clear();
	result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);
 
       return result;
    }

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

    return false;
}

bool SearchController::HasWork() {
    return true;
}




