#ifndef DROPOFCONTROLLER_H
#define DROPOFCONTROLLER_H
#define HEADERFILE_H

#include <geometry_msgs/Pose2D.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <std_msgs/Float32.h>
#include <ros/ros.h>

#include "Controller.h"
#include "StandardVars.h"

class DropOffController : virtual Controller
{
public:
    DropOffController();
    ~DropOffController();

    void Reset() override;
    Result DoWork() override;
    bool ShouldInterrupt() override;
    bool HasWork() override;

    bool IsChangingMode();
    void SetLocationData(geometry_msgs::Pose2D center, geometry_msgs::Pose2D current);
    void SetTargetPickedUp();
    void SetBlockBlockingUltrasound(bool blockBlock);
    void setTargetData(const apriltags_ros::AprilTagDetectionArray::ConstPtr& message);

    float GetSpinner() {return spinner;}


private:

    void ProcessData();

    //Constants

    const float cameraOffsetCorrection = 0.020; //meters
    const float centeringTurn = 0.15; //radians
    const int centerTagThreshold = 10;
    const int centerTagSearchCutoff = 6; //seconds
    const float collectionPointVisualDistance = 0.3; //in meters
    const float initialSpinSize = 0.10; //in meters aka 10cm
    const float spinSizeIncrement = 0.10; //in meters
    const float searchVelocity = 0.15; //in meters per second



    //Instance Variables

    /*
     *  Timers and Accumulators
     */

    //keep track of progression around a circle when driving in a circle
    float spinner;

    //Timer for return code (dropping the cube in the center)- used for timerTimeElapsed
    ros::Time returnTimer;

    //Time since last exceeding the tag threshold
    ros::Time lastCenterTagThresholdTime;

    //Previous tag count
    int prevCount;


    /*
     *  Cached External Information
     */

    //Count of tags on the left and right, respectively
    int countLeft;
    int countRight;

    //Center and current locations as of the last call to setLocationData
    geometry_msgs::Pose2D centerLocation;
    geometry_msgs::Pose2D currentLocation;

    //Time since modeTimer was started, in seconds
    float timerTimeElapsed;

    //The amount over initialSpinSize we've gotten to
    float spinSizeIncrease;

    /*
     *  Flags
     */

    //Flag indicating that a target has been picked up and is held
    bool targetHeld;

    //Flag indicating that we're in the center
    bool reachedCollectionPoint;

    //Flag indicating that we're driving in a circle to find the nest
    bool circularCenterSearching;

    //Flag for when we are entering the center circle
    bool centerApproach;

    //we have seen enough central collection tags to be certain we are either in or driving towards the nest.
    bool seenEnoughCenterTags;

    //Flag to indicate a switch to precision driving
    bool isPrecisionDriving;

    //Flag to indicate that we're starting to follow waypoints
    bool startWaypoint;

    Result result;


};
#endif // end header define
