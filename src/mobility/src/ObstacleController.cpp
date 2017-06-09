#include "ObstacleController.h"

ObstacleController::ObstacleController()
{
    
}

Result ObstacleController::CalculateResult() {
    geometry_msgs::Pose2D obstacleAvoidLocation;
    
    if (right < 0.8) {
        // select new heading 0.2 radians to the left
        obstacleAvoidLocation.theta = currentLocation.theta + 0.6;
    }
 
    // obstacle in front or on left side
    else if (left < 0.8 || center < 0.8) {
        // select new heading 0.2 radians to the right
        obstacleAvoidLocation.theta = currentLocation.theta + 0.6;
    }
    
    obstacleAvoidLocation.x = currentLocation.x + (0.50 * cos(obstacleAvoidLocation.theta));
    obstacleAvoidLocation.y = currentLocation.y + (0.50 * sin(obstacleAvoidLocation.theta));
    
    result.wpts.waypoints.at(0) = obstacleAvoidLocation;
    
    result.type = waypoint;
    
    return result;
}

void ObstacleController::UpdateData(float left, float center, float right ,geometry_msgs::Pose2D currentLocation) {
    
    this->left = left;
    this->right = right;
    this->center = center;
    
   if (left < 0.8 || right < 0.8 || center < 0.8) {
       obstacleInterrupt = true;
   }
   else {
       obstacleInterrupt = false;
   }  
   this->currentLocation = currentLocation;
   
}

bool ObstacleController::ShouldInterrupt() {
    return obstacleInterrupt;
}
