#ifndef DROPOFCONTROLLER_H
#define DROPOFCONTROLLER_H
#define HEADERFILE_H

#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float32.h>

struct DropOffResult {
    float cmdVel;
    float angleError;
    float fingerAngle;
    float wristAngle;
    bool goalDriving; //set true for when driving is goal location oriented and false when it is target or time oriented
    geometry_msgs::Pose2D centerGoal;
    bool reset;
    bool timer;
};

class DropOffController
{
public:
    DropOffController();
    ~DropOffController();

    void reset();

    DropOffResult getState() {return result;}
    float getSpinner() {return spinner;}
    float getCentX() { return centerLocation.x;}
    float getCount() { return count;}

    void setDataTargets(int ccount, double lleft, double rright);
    void setCenterDist(float dist) {distanceToCenter = dist;}
    void setDataLocations(geometry_msgs::Pose2D center, geometry_msgs::Pose2D current, float sync);

private:

    bool right;
    bool left;
    bool reachedCollectionPoint;

    //is driving in a circle to find the nest
    bool circularCenterSearching;

    //keep track of progression around a circle when driving in a circle
    float spinner;

    //set to true when we are entering the center circle
    bool centerApproach;

    //we have seen enough central collection tags to be certain we are either in or driving towards the nest.
    bool seenEnoughCenterTags;

    //central collection point has been seen (aka the nest)
    bool centerSeen;

    time_t timeWithoutSeeingEnoughCenterTags;
    float cameraOffsetCorrection;
    float centeringTurn;
    int seenEnoughCenterTagsCount;
    DropOffResult result;
    int count;
    int prevCount;
    double countLeft;
    double countRight;
    float collectionPointVisualDistance;
    float distanceToCenter;
    geometry_msgs::Pose2D centerLocation;
    geometry_msgs::Pose2D currentLocation;
    float timerTimeElapsed;
    time_t timerStartTime;
    float timeElapsedSinceTimeSinceSeeingEnoughCenterTags;
    float spinSize;
    float addSpinSize;
    float addSpinSizeAmmount;

    float searchVelocity;

    void calculateDecision();


};
#endif // end header define
