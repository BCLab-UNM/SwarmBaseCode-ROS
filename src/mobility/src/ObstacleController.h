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

    /*
     * Hand-tuned constants to use in precision driving for obstacle avoidance
     */


    const float K_linear_target = 0.02143f;
    const float K_angular_target = 0.01500f;

    const float K_linear_obstacle = 0.4384;
    const float K_angular_obstacle = 0.033;

    const int targetCountPivot = 6;
    const float obstacleDistancePivot = 0.2526;

    /*
     * Member variables
     */


    bool obstacleInterrupt;
    
    float left = 0;
    float center = 0;
    float right = 0;

    float velocityEstimate = 0;

    int countLeft;
    int countRight;
    bool centerSeen;


    geometry_msgs::Pose2D currentLocation;
    
};

#endif // OBSTACLECONTOLLER_H
