#ifndef PICKUPCONTROLLER_H
#define PICKUPCONTROLLER_H

#include <ros/ros.h>
#include "Controller.h"
#include "TagPoint.h"

class PickUpController : virtual Controller
{
public:
    PickUpController();
    ~PickUpController();

    void Reset() override;
    Result DoWork() override;
    void SetTagData(vector<TagPoint> tags);
    bool ShouldInterrupt() override;
    bool HasWork() override;

    bool SetSonarData(float rangeCenter);

    float getDistance() {return blockDistance;}
    bool GetLockTarget() {return lockTarget;}
    float GetTD() {return timeDifference;}
    void SetUltraSoundData(bool blockBlock);

    bool GetIgnoreCenter() {return ignoreCenterSonar;}

protected:

    void ProcessData();

private:
    //set true when the target block is less than targetDist so we continue attempting to pick it up rather than
    //switching to another block that is in view
    bool lockTarget;

    bool targetFound;

    // Failsafe state. No legitimate behavior state. If in this state for too long return to searching as default behavior.
    bool timeOut;
    int nTargetsSeen;
    ros::Time millTimer;

    //yaw error to target block
    double blockYawError;

    //distance to target block from front of robot
    double blockDistance;

    //distance to target block from camera
    double blockCameraDistance;

    //struct for returning data to the ROS adapter
    Result result;

    bool blockBlock;

    bool ignoreCenterSonar = false;

    float timeDifference;


};
#endif // end header define
