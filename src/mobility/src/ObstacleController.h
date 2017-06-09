#ifndef OBSTACLECONTOLLER_H
#define OBSTACLECONTOLLER_H

#include "StandardVars.h"
#include <geometry_msgs/Pose2D.h>

class ObstacleController
{
public:
    ObstacleController();
    
    Result result;
    
    Result CalculateResult();
    
    void UpdateData(float left, float center, float right, geometry_msgs::Pose2D currentLocation);
    bool ShouldInterrupt();
    
private:
    bool obstacleInterrupt;
    
    float left = 0;
    float center = 0;
    float right = 0;

    geometry_msgs::Pose2D currentLocation;
    
};

#endif // OBSTACLECONTOLLER_H
