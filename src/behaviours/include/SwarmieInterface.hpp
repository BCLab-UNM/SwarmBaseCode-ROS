#ifndef _SWARMIE_INTERFACE_HPP
#define _SWARMIE_INTERFACE_HPP

#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <apriltags_ros/AprilTagDetectionArray.h>

#include "RobotInterface.hpp"
#include "SwarmieSensors.hpp"
#include "Action.hpp"

enum class WristControl   { UP, DOWN, DOWN_2_3 };
enum class GripperControl { OPEN, CLOSED };

class SwarmieAction : public core::Action
{
private:
   WristControl   _wrist;
   GripperControl _grip;
public:
   SwarmieAction() : core::Action(core::VelocityAction()) {}
   SwarmieAction(core::VelocityAction va) : core::Action(va) {}
   ~SwarmieAction() {}
   GripperControl GripperCommand() const { return _grip; }
   WristControl   WristCommand()   const { return _wrist; }
   void SetGrip(GripperControl g);
   void SetWrist(WristControl w);
};

class SwarmieInterface : public core::RobotInterface
{
private:
   ros::NodeHandle _nh;
   ros::Publisher  _drivePublisher;
   ros::Publisher  _gripPublisher;
   ros::Publisher  _wristPublisher;
   ros::Subscriber _leftSubscriber;
   ros::Subscriber _rightSubscriber;
   ros::Subscriber _centerSubscriber;
   ros::Subscriber _tagSubscriber;
   ros::Subscriber _odomSubscriber;
   ros::Subscriber _gpsFusedSubscriber;

   SwarmieSensors _sensors;

   const float WRIST_UP_ANGLE = 0;
   const float WRIST_DOWN_ANGLE = 1.25;
   const float GRIP_OPEN_ANGLE = 3.14159/2;
   const float GRIP_CLOSED_ANGLE = 0;

   void TagHandler(const apriltags_ros::AprilTagDetectionArray::ConstPtr& message);
   void LeftSonarHandler(const sensor_msgs::Range& range);
   void CenterSonarHandler(const sensor_msgs::Range& range);
   void RightSonarHandler(const sensor_msgs::Range& range);
   void OdometryHandler(const nav_msgs::Odometry::ConstPtr& odom);
   void GPSFusedHandler(const nav_msgs::Odometry::ConstPtr& odom);
public:
   SwarmieInterface(std::string name);
   ~SwarmieInterface() {}

   const core::Sensors& GetSensors() override { return _sensors; }
   void DoAction(const core::Action& a) override;
};

#endif // _ROBOT_INTERFACE_HPP
