#include "ObstacleController.h"

ObstacleController::ObstacleController()
{
    
}

Result ObstacleController::CalculateResult() {
    geometry_msgs::Pose2D obstacleAvoidLocation;
    float centeringTurn = 0.15; //radians

    if(centerSeen){
        // Turn left if more tags are seen on the right
        if (countRight > countLeft){
            obstacleAvoidLocation.theta += centeringTurn;

        // Turn Right if more tags are seen on the left
        } else {
            obstacleAvoidLocation.theta -=centeringTurn;
        }

        obstacleAvoidLocation.x = currentLocation.x + (.50 *cos(obstacleAvoidLocation.theta));
        obstacleAvoidLocation.y = currentLocation.y + (.50 *sin(obstacleAvoidLocation.theta));

        result.wpts.waypoints.insert(result.wpts.waypoints.begin(), obstacleAvoidLocation);

        result.type = waypoint;

        return result;


    }else{
    
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

void ObstacleController::UpdateData(const apriltags_ros::AprilTagDetectionArray::ConstPtr& message){
    float cameraOffsetCorrection = 0.020; //meters;
    centerSeen = false;
    // this loop is to get the number of center tags
    for (int i = 0; i < message->detections.size(); i++) {
        if (message->detections[i].id == 256) {
            geometry_msgs::PoseStamped cenPose = message->detections[i].pose;

            // checks if tag is on the right or left side of the image
            if (cenPose.pose.position.x + cameraOffsetCorrection > 0) {
                countRight++;

            } else {
                countLeft++;
            }
            centerSeen = true;
        }
    }
}

bool ObstacleController::ShouldInterrupt() {
    return obstacleInterrupt;
}





