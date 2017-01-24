#include "PickUpController.h"

PickUpController::PickUpController() {
    lockTarget = false;
    timeOut = false;
    nTargetsSeen = 0;
    blockYawError = 0;
    blockDist = 0;
    td = 0;

    result.pickedUp = false;
    result.cmdVel = 0;
    result.angleError = 0;
    result.fingerAngle = -1;
    result.wristAngle = -1;
    result.giveUp = false;

}

PickUpResult PickUpController::pickUpSelectedTarget(bool blockBlock) {
    //threshold distance to be from the target block before attempting pickup
    float targetDist = 0.25; //meters


    /*PickUpResult result;
  result.pickedUp = false;
  result.cmdVel = 0;
  result.angleError = 0;
  result.fingerAngle = -1;
  result.wristAngle = -1;
  result.pickedUp = false;
  result.giveUp = false;*/

    // millisecond time = current time if not in a counting state
    if (!timeOut) millTimer = ros::Time::now();

    //diffrence between current time and millisecond time
    ros::Duration Tdiff = ros::Time::now() - millTimer;
    float Td = Tdiff.sec + Tdiff.nsec/1000000000.0;
    td = Td;

    if (nTargetsSeen == 0 && !lockTarget) //if not targets detected and a target has not been locked in
    {
        if(!timeOut) //if not in a counting state
        {
            result.cmdVel = 0.0;
            result.angleError = 0.0;

            timeOut = true;
        }
        //if in a counting state and has been counting for 1 second
        else if (Td > 1 && Td < 2.5)
        {
            result.cmdVel = -0.2;
            result.angleError = 0.0;
        }
    }
    else if (blockDist > targetDist && !lockTarget) //if a target is detected but not locked, and not too close.
    {
        float vel = blockDist * 0.20;
        if (vel < 0.1) vel = 0.1;
        if (vel > 0.2) vel = 0.2;
        result.cmdVel = vel;
        result.angleError = -blockYawError/2;
        timeOut = false;
        nTargetsSeen = 0;
        return result;
    }
    else if (!lockTarget) //if a target hasn't been locked lock it and enter a counting state while slowly driving forward.
    {
        lockTarget = true;
        result.cmdVel = 0.18;
        result.angleError = 0.0;
        timeOut = true;
    }
    else if (Td > 2.4) //raise the wrist
    {
        result.cmdVel = -0.25;
        result.angleError = 0.0;
        result.wristAngle = 0;
    }
    else if (Td > 1.7) //close the fingers and stop driving
    {
        result.cmdVel = -0.1;
        result.angleError = 0.0;
        result.fingerAngle = 0;
        return result;
    }

    if (Td > 3.8 && timeOut) //if enough time has passed enter a recovery state to re-attempt a pickup
    {
        if (blockBlock) //if the ultrasound is blocked at less than .12 meters a block has been picked up no new pickup required
        {
            result.pickedUp = true;
        }
        else //recover begin looking for targets again
        {
            lockTarget = false;
            result.cmdVel = -0.15;
            result.angleError = 0.0;
            //set gripper
            result.fingerAngle = M_PI_2;
            result.wristAngle = 0;
        }
    }

    if (Td > 5 && timeOut) //if no targets are found after too long a period go back to search pattern
    {
        result.giveUp = true;
        lockTarget = false;
        timeOut = false;
        result.cmdVel = 0.0;
        result.angleError = 0.0;
    }

    return result;
}

PickUpResult PickUpController::selectTarget(const apriltags_ros::AprilTagDetectionArray::ConstPtr& message) {

    /*PickUpResult result;
  result.pickedUp = false;
  result.cmdVel = 0;
  result.angleError = 0;
  result.fingerAngle = -1;
  result.wristAngle = -1;
  result.giveUp = false;*/


    nTargetsSeen = 0;
    nTargetsSeen = message->detections.size();

    double closest = std::numeric_limits<double>::max();
    int target  = 0;
    for (int i = 0; i < message->detections.size(); i++) //this loop selects the closest visible block to makes goals for it
    {
        geometry_msgs::PoseStamped tagPose = message->detections[i].pose;
        double test = hypot(hypot(tagPose.pose.position.x, tagPose.pose.position.y), tagPose.pose.position.z); //absolute distance to block from camera lense

        if (closest > test)
        {
            target = i;
            closest = test;
            blockDist = hypot(tagPose.pose.position.z, tagPose.pose.position.y); //distance from bottom center of chassis ignoring height.
            blockDist = sqrt(blockDist*blockDist - 0.195*0.195);
            blockYawError = atan((tagPose.pose.position.x + 0.020)/blockDist)*1.05; //angle to block from bottom center of chassis on the horizontal.
        }
    }
    if ( blockYawError > 10) blockYawError = 10; //limits block angle error to prevent overspeed from PID.
    if ( blockYawError < - 10) blockYawError = -10; //due to detetionropping out when moveing quickly

    geometry_msgs::PoseStamped tagPose = message->detections[target].pose;

    //if target is close enough
    //diffrence between current time and millisecond time
    ros::Duration Tdiff = ros::Time::now() - millTimer;
    float Td = Tdiff.sec + Tdiff.nsec/1000000000;

    if (hypot(hypot(tagPose.pose.position.x, tagPose.pose.position.y), tagPose.pose.position.z) < 0.13 && Td < 3.8) {
        result.pickedUp = true;
    }

    //Lower wrist and open fingures if no locked targt
    else if (!lockTarget)
    {

        //set gripper;
        result.fingerAngle = M_PI_2;
        result.wristAngle = 1.25;
    }

    return result;
}

void PickUpController::reset() {
    result.pickedUp = false;
    lockTarget = false;
    timeOut = false;
    nTargetsSeen = 0;
    blockYawError = 0;
    blockDist = 0;
    td = 0;

    result.pickedUp = false;
    result.cmdVel = 0;
    result.angleError = 0;
    result.fingerAngle = -1;
    result.wristAngle = -1;
    result.giveUp = false;
}

PickUpController::~PickUpController() {
}
