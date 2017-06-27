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
    cout << "in obstacle controller" << endl;

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


void ObstacleController::SetSonarData(float sonarleft, float sonarcenter, float sonarright) {
    left = sonarleft;
    right = sonarright;
    center = sonarcenter;

    ProcessData();
}

void ObstacleController::SetCurrentLocation(Point currentLocation) {
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

    if (left < triggerDistance || right < triggerDistance || center < triggerDistance || centerSeen) {
        obstacleDetected = true;
        obstacleAvoided = false;
    } else {
        obstacleAvoided = true;
    }
}

void ObstacleController::SetTagData(vector<TagPoint> tags){
    float cameraOffsetCorrection = 0.020; //meters;
    centerSeen = false;
    // this loop is to get the number of center tags
    for (int i = 0; i < tags.size(); i++) {
        if (tags[i].id == 256) {

            // checks if tag is on the right or left side of the image
            if (tags[i].x + cameraOffsetCorrection > 0) {
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
        if(obstacleAvoided && obstacleDetected) {
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
    ignoreCenter = true; //ignore center ultrasound
}
