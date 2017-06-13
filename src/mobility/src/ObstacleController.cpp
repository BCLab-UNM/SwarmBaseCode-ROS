#include "ObstacleController.h"

ObstacleController::ObstacleController()
{
    
}

Result ObstacleController::CalculateResult() {

    if(centerSeen){

        result.type = precisionDriving;

        int total = countLeft + countRight;
        int difference = countRight - countLeft;
        float absDifference = std::abs((float)(difference));

        /* targetCountPivot is the number of targets seen where the rover will go from
         * driving forwards if total < targetCountPivot,
         * stopping motion if total == targetCountPivot,
         * driving backwards if total > targetCountPivot
         *
         * This formula results in behavior that appears as turning slowly while moving forward,
         * stopping and turning in place if the rover gets too close,
         * and backing up while turning away
         *
         * If this causes rovers to be trapped in the center, a new stepwise function should be defined
         * where the speed decreases until the pivot point, then is totally stopped when there are
         * as many or more targets in sight than the pivot point.
         */

        result.pd.cmdVel = -(total - targetCountPivot) * K_linear_target;


        //This formula scales turning so that it is faster when more targets are visible
        //Turns right if there are an even number of targets on either side of the rover's vision
        if(difference != 0) {
            result.pd.cmdAngularError = (difference / absDifference) * total * K_angular_target;
        } else {
            result.pd.cmdAngularError = -total * K_angular_target;
        }

        result.pd.setPointVel = result.pd.cmdVel;
        result.pd.setPointYaw = (currentLocation.theta + result.pd.cmdAngularError);

        return result;

    } else {

        //obstacle on right side
        if (right < 0.8) {
            result.type = precisionDriving;

            result.pd.cmdVel = (1.0 - obstacleDistancePivot/right) * K_linear_obstacle;
            result.pd.cmdAngularError = (1.0 / right) * K_angular_obstacle;
            result.pd.setPointVel = result.pd.cmdVel;
            result.pd.setPointYaw = currentLocation.theta + result.pd.cmdAngularError;
        }// obstacle in front or on left side
        else if (left < 0.8 || center < 0.8) {
            result.type = precisionDriving;

            result.pd.cmdVel = (1.0 - obstacleDistancePivot/right) * K_linear_obstacle;
            result.pd.cmdAngularError = -(1.0 / right) * K_angular_obstacle;
            result.pd.setPointVel = result.pd.cmdVel;
            result.pd.setPointYaw = currentLocation.theta + result.pd.cmdAngularError;
        } //no more obstacles, including tags- set waypoint such that you continue on just a little further
        else {
            if(result.type != waypoint) {
                result.type = waypoint;

                geometry_msgs::Pose2D obstacleAvoidLocation;

                float finalTheta = currentLocation.theta + result.pd.cmdAngularError;
                obstacleAvoidLocation.x = 0.3 * cos(finalTheta);
                obstacleAvoidLocation.y = 0.3 * sin(finalTheta);
                obstacleAvoidLocation.theta = finalTheta;
            } //we've reached our previously-set waypoint, time to relinquish control
            else {

                result.type = behavior;
                result.b = obstacleAvoided;

            }
        }

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

   velocityEstimate = hypot(currentLocation.x - this->currentLocation.x, currentLocation.y - this->currentLocation.y) / 0.1;
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





