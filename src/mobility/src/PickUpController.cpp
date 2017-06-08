#include "PickUpController.h"

PickUpController::PickUpController() {
    lockTarget = false;
    timeOut = false;
    nTargetsSeen = 0;
    blockYawError = 0;
    blockDistance = 0;
    timeDifference = 0;
    
    result.type;
    result.pd.cmdVel = 0;
    result.pd.cmdAngular = 0;
    result.fingerAngle = -1;
    result.wristAngle = -1;
    
}

Result PickUpController::run() {
    if (!targetCollected) {
        result.type = precisionDriving;
        
        //pickUpSelectedTarget();
    }
    else {
        
    }
    
    return result;
    
}

void PickUpController::UpdateData(const apriltags_ros::AprilTagDetectionArray::ConstPtr &message) {
    
    if (message->detections.size() > 0) {
        
        float cameraOffsetCorrection = 0.020; //meters;
        
        nTargetsSeen = message->detections.size();
        
        double closest = std::numeric_limits<double>::max();
        int target  = 0;
        for (int i = 0; i < message->detections.size(); i++) { //this loop selects the closest visible block to makes goals for it
        
            if (message->detections[i].id == 0) {
                
                geometry_msgs::PoseStamped tagPose = message->detections[i].pose;
                double test = hypot(hypot(tagPose.pose.position.x, tagPose.pose.position.y), tagPose.pose.position.z); //absolute distance to block from camera lense
                if (closest > test)
                {
                    target = i;
                    closest = test;
                    targetFound = true;
                }
            }
            else {
                nTargetsSeen--;
            }
        }
        
        geometry_msgs::PoseStamped tagPose = message->detections[target].pose;
                
        blockDistance = hypot(tagPose.pose.position.z, tagPose.pose.position.y); //distance from bottom center of chassis ignoring height.
        if ( (blockDistance*blockDistance - 0.195*0.195) > 0 )
        {
            blockDistance = sqrt(blockDistance*blockDistance - 0.195*0.195);
        }
        else
        {
            float epsilon = 0.00001; // A small non-zero positive number
            blockDistance = epsilon;
        }
        blockYawError = atan((tagPose.pose.position.x + cameraOffsetCorrection)/blockDistance)*1.05; //angle to block from bottom center of chassis on the horizontal.

        if(blockYawError > 10) blockYawError =10;
        if(blockYawError < -10) blockYawError = -10;


        

        
        //if target is close enough
        //diffrence between current time and millisecond time
        ros::Duration Tdiff = ros::Time::now() - millTimer;
        float Td = Tdiff.sec + Tdiff.nsec/1000000000;
     
        if (hypot(hypot(tagPose.pose.position.x, tagPose.pose.position.y), tagPose.pose.position.z) < 0.13 && Td < 3.8) {
            result.type = behavior;
            result.b = targetPickedUp;
        }
        
        //Lower wrist and open fingures if no locked targt
        else if (!lockTarget)
        {
            //set gripper;
            result.fingerAngle = M_PI_2;
            result.wristAngle = 1.25;
        }
    }


}


bool PickUpController::ShouldInterrupt(){
    return targetFound;
}



Result PickUpController::CalculateResult() {      //****not named correctly and needs to return a properly formated result and be called CalculateResult that takes in no parameters
                                                                    //instead blockBlock will be passaed in through a new method such as setUltraSoundData
    //threshold distance to be from the target block before attempting pickup
    float targetDistance = 0.25; //meters
    
    
    // millisecond time = current time if not in a counting state
    if (!timeOut) millTimer = ros::Time::now();
    
    //diffrence between current time and millisecond time
    ros::Duration Tdifference = ros::Time::now() - millTimer;
    float Td = Tdifference.sec + Tdifference.nsec/1000000000.0;
    timeDifference = Td;
    
    if (nTargetsSeen == 0 && !lockTarget) //if not targets detected and a target has not been locked in
    {
        if(!timeOut) //if not in a counting state
        {
            result.pd.cmdVel = 0.0;
            result.pd.cmdAngular = 0.0;
            
            timeOut = true;
        }
        //if in a counting state and has been counting for 1 second
        else if (Td > 1 && Td < 2.5)
        {
            result.pd.cmdVel = -0.2;
            result.pd.cmdAngular = 0.0;
        }
    }
    else if (blockDistance > targetDistance && !lockTarget) //if a target is detected but not locked, and not too close.
    {
        float vel = blockDistance * 0.20;
        if (vel < 0.1) vel = 0.1;
        if (vel > 0.2) vel = 0.2;
        result.pd.cmdVel = vel;
        result.pd.cmdAngular = -blockYawError/2;
        timeOut = false;
        nTargetsSeen = 0;
        return result;
    }
    else if (!lockTarget) //if a target hasn't been locked lock it and enter a counting state while slowly driving forward.
    {
        lockTarget = true;
        result.pd.cmdVel = 0.18;
        result.pd.cmdAngular = 0.0;
        timeOut = true;
    }
    else if (Td > 2.4) //raise the wrist
    {
        result.pd.cmdVel = -0.25;
        result.pd.cmdAngular = 0.0;
        result.wristAngle = 0;
    }
    else if (Td > 1.7) //close the fingers and stop driving
    {
        result.pd.cmdVel = -0.1;
        result.pd.cmdAngular = 0.0;
        result.fingerAngle = 0;
        return result;
    }
    
    if (Td > 3.8 && timeOut) //if enough time has passed enter a recovery state to re-attempt a pickup
    {
        if (blockBlock) //if the ultrasound is blocked at less than .12 meters a block has been picked up no new pickup required
        {
            result.type = behavior;
            result.b = targetPickedUp;
        }
        else //recover begin looking for targets again
        {
            lockTarget = false;
            result.pd.cmdVel = -0.15;
            result.pd.cmdAngular = 0.0;
            //set gripper
            result.fingerAngle = M_PI_2;
            result.wristAngle = 0;
        }
    }
    
    if (Td > 5 && timeOut) //if no targets are found after too long a period go back to search pattern
    {
        result.type = behavior;
        result.b = targetLost;
        lockTarget = false;
        timeOut = false;
        result.pd.cmdVel = 0.0;
        result.pd.cmdAngular = 0.0;
    }

        return result;
}

void PickUpController::reset() {
    
    result.type = behavior;
    result.b = init;
    lockTarget = false;
    timeOut = false;
    nTargetsSeen = 0;
    blockYawError = 0;
    blockDistance = 0;
    timeDifference = 0;
    
    result.pd.cmdVel = 0;
    result.pd.cmdAngular = 0;
    result.fingerAngle = -1;
    result.wristAngle = -1;
};

void PickUpController::SetUltraSoundData(bool blockBlock){
    this->blockBlock = blockBlock;
}



PickUpController::~PickUpController() {
}
