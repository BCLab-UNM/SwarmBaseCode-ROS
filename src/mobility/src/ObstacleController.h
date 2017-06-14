#ifndef OBSTACLECONTOLLER_H
#define OBSTACLECONTOLLER_H

#include "StandardVars.h"
#include <geometry_msgs/Pose2D.h>
#include <apriltags_ros/AprilTagDetectionArray.h>


class ObstacleController
{
public:
    ObstacleController();
    
    Result result;
    
    Result CalculateResult();
    
    void UpdateData(float left, float center, float right, geometry_msgs::Pose2D currentLocation);
    bool ShouldInterrupt();
    void UpdateData(const apriltags_ros::AprilTagDetectionArray::ConstPtr& message);
    void SetIgnoreCenter();

    
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
