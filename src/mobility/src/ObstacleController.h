#ifndef OBSTACLECONTOLLER_H
#define OBSTACLECONTOLLER_H

#include "StandardVars.h"
#include "Controller.h"
#include <geometry_msgs/Pose2D.h>
#include <apriltags_ros/AprilTagDetectionArray.h>


class ObstacleController : virtual Controller
{
public:
    ObstacleController();
    
    Result result;
    
    void Reset() override;
    Result DoWork() override;
    void UpdateData(float left, float center, float right, geometry_msgs::Pose2D currentLocation);
    void UpdateData(const apriltags_ros::AprilTagDetectionArray::ConstPtr& message);
    bool ShouldInterrupt() override;
    bool HasWork() override;
    void SetIgnoreCenter();

protected:

    void ProcessData();

private:

    const float K_angular = 0.01500f;
    const float reactivateCenterThreshold = 0.8;
    const int targetCountPivot = 6;
    const float obstacleDistancePivot = 0.2526;
    const float triggerDistance = 0.8;

    /*
     * Member variables
     */


    bool obstacleInterrupt;
    bool obstacleDetected;
    bool obstacleAvoided;
    
    float left = 0;
    float center = 0;
    float right = 0;

    int countLeft;
    int countRight;
    bool centerSeen;

    bool ignoreCenter = false;


    geometry_msgs::Pose2D currentLocation;
    
};

#endif // OBSTACLECONTOLLER_H
