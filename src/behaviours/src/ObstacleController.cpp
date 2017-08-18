#include "ObstacleController.h"

ObstacleController::ObstacleController()
{
  obstacleAvoided = true;
  obstacleDetected = false;
  obstacleInterrupt = false;
  result.PIDMode = CONST_PID;
}

void ObstacleController::Reset() {
  obstacleAvoided = true;
  obstacleDetected = false;
  obstacleInterrupt = false;
  delay = current_time;
}

Result ObstacleController::DoWork() {

  clearWaypoints = true;
  set_waypoint = true;
  result.PIDMode = CONST_PID;

  if(centerSeen){


    result.type = precisionDriving;

    result.pd.cmdVel = 0.0;

    if(countLeft < countRight) {
      result.pd.cmdAngular = K_angular;
    } else {
      result.pd.cmdAngular = -K_angular;
    }

    result.pd.setPointVel = 0.0;
    result.pd.cmdVel = 0.0;
    result.pd.setPointYaw = 0;

  }
  else {


    //obstacle on right side
    if (right < 0.8 || center < 0.8 || left < 0.8) {
      result.type = precisionDriving;

      result.pd.cmdAngular = -K_angular;

      result.pd.setPointVel = 0.0;
      result.pd.cmdVel = 0.0;
      result.pd.setPointYaw = 0;
    }
  }

  if (can_set_waypoint) {

    can_set_waypoint = false;
    set_waypoint = false;
    clearWaypoints = false;

    result.type = waypoint;
    result.PIDMode = FAST_PID;
    Point forward;
    forward.x = currentLocation.x + (0.5 * cos(currentLocation.theta));
    forward.y = currentLocation.y + (0.5 * sin(currentLocation.theta));
    result.wpts.waypoints.clear();
    result.wpts.waypoints.push_back(forward);
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

  //timeout timer for no tag messages
  long int Tdifference = current_time - timeSinceTags;
  float Td = Tdifference/1e3;
  if (Td >= 0.5) {
    centerSeen = false;
    phys= false;
    if (!obstacleAvoided)
    {
      can_set_waypoint = true;
    }
  }

  //Process sonar info
  if(ignoreCenter){
    if(center > reactivateCenterThreshold){
      ignoreCenter = false;
    }
    else{
      center = 3;
    }
  }
  else {
    if (center < 0.12) {
      result.wristAngle = 0.7;
    }
    else {
      result.wristAngle = -1;
    }
  }

  if (left < triggerDistance || right < triggerDistance || center < triggerDistance)
  {
    phys = true;
    timeSinceTags = current_time;
  }


  if (centerSeen || phys)
  {
    obstacleDetected = true;
    obstacleAvoided = false;
    can_set_waypoint = false;
  }
  else
  {
    obstacleAvoided = true;
  }
}

void ObstacleController::SetTagData(vector<TagPoint> tags){
  float cameraOffsetCorrection = 0.020; //meters;
  centerSeen = false;
  countLeft = 0;
  countRight = 0;

  // this loop is to get the number of center tags
  if (!targetHeld) {
    for (int i = 0; i < tags.size(); i++) {
      if (tags[i].id == 256) {

        // checks if tag is on the right or left side of the image
        if (tags[i].x + cameraOffsetCorrection > 0) {
          countRight++;

        } else {
          countLeft++;
        }
        centerSeen = true;
        timeSinceTags = current_time;
      }
    }
  }

}

bool ObstacleController::ShouldInterrupt() {


  if(obstacleDetected && !obstacleInterrupt)
  {
    obstacleInterrupt = true;
    return true;
  }
  else
  {
    if(obstacleAvoided && obstacleDetected)
    {
      Reset();
      return true;
    } else {
      return false;
    }
  }
}

bool ObstacleController::HasWork() {
  if (can_set_waypoint && set_waypoint)
  {
    return true;
  }

  return !obstacleAvoided;
}

void ObstacleController::SetIgnoreCenter(){
  ignoreCenter = true; //ignore center ultrasound
}

void ObstacleController::SetCurrentTimeInMilliSecs( long int time )
{
  current_time = time;
}

void ObstacleController::SetTargetHeld() {
  targetHeld = true;


  if (previousTargetState == false) {
    obstacleAvoided = true;
    obstacleInterrupt = false;
    obstacleDetected = false;
    previousTargetState = true;
  }
}
