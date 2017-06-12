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

    
private:
    bool obstacleInterrupt;
    
    float left = 0;
    float center = 0;
    float right = 0;

    int countLeft;
    int countRight;
    bool centerSeen;


    geometry_msgs::Pose2D currentLocation;
    
};

#endif // OBSTACLECONTOLLER_H
