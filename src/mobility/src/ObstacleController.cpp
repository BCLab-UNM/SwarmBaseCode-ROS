#include "ObstacleController.h"

ObstacleController::ObstacleController()
{
    obstacleAvoided = true;
    obstacleDetected = false;
    obstacleInterrupt = false;
}

void ObstacleController::Reset() {
    obstacleAvoided = true;
    obstacleDetected = false;
    obstacleInterrupt = false;
}

Result ObstacleController::DoWork() {

    if(centerSeen){

        result.type = precisionDriving;

        result.pd.cmdVel = 0.0;

        if(countLeft < countRight) {
            result.pd.setPointYaw = -K_angular;
        } else {
            result.pd.setPointYaw = K_angular;
        }

        result.pd.setPointVel = 0.0;
        result.pd.cmdVel = 0.0;
        result.pd.cmdAngularError = (currentLocation.theta + result.pd.setPointYaw);

    }
    else {

        //obstacle on right side
        if (right < 0.8 || center < 0.8 || left < 0.8) {
            result.type = precisionDriving;

            result.pd.setPointYaw = -K_angular;

            result.pd.setPointVel = 0.0;
            result.pd.cmdVel = 0.0;
            result.pd.cmdAngularError = (currentLocation.theta + result.pd.setPointYaw);
        }
    }

    return result;
}


void ObstacleController::UpdateData(float sonarleft, float sonarcenter, float sonarright ,geometry_msgs::Pose2D currentLocation) {
    
    left = sonarleft;
    right = sonarright;
    center = sonarcenter;

    this->currentLocation = currentLocation;

}

void ObstacleController::ProcessData() {

    //Process sonar info
    if(ignoreCenter){
        if(center > reactivateCenterThreshold){
            ignoreCenter = false;

        }
        else{
            center = 3;


        }
    }

    if ((left < triggerDistance || right < triggerDistance || center < triggerDistance || centerSeen) && !obstacleInterrupt) {
        obstacleDetected = true;
        obstacleAvoided = false;
    } else {
        obstacleAvoided = true;
    }
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
    ProcessData();

    if(obstacleDetected && !obstacleInterrupt) {
        obstacleInterrupt = true;
        return true;
    } else {
        if(obstacleAvoided) {
            Reset();
            return true;
        } else {
            return false;
        }
    }
}

bool ObstacleController::HasWork() {
    return !obstacleAvoided;
}

void ObstacleController::SetIgnoreCenter(){
    ignoreCenter = true;
}
