#ifndef _SWARMIE_INTERFACE_HPP
#define _SWARMIE_INTERFACE_HPP

#include <ros/ros.h>
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

class SwarmieInterface
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

   SwarmieSensors _sensors;

   const float WRIST_UP_ANGLE = 0;
   const float WRIST_DOWN_ANGLE = 1.25;
   const float GRIP_OPEN_ANGLE = 3.14159/2;
   const float GRIP_CLOSED_ANGLE = 0;

   void TagHandler(const apriltags_ros::AprilTagDetectionArray::ConstPtr& message);
   void LeftSonarHandler(const sensor_msgs::Range& range);
   void CenterSonarHandler(const sensor_msgs::Range& range);
   void RightSonarHandler(const sensor_msgs::Range& range);
public:
   SwarmieInterface(std::string name);
   ~SwarmieInterface() {}

   const SwarmieSensors* GetSensors() { return &_sensors; }
   void DoAction(const SwarmieAction& a);
};

#endif // _ROBOT_INTERFACE_HPP
