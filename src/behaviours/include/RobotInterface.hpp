#ifndef _ROBOT_INTERFACE_HPP
#define _ROBOT_INTERFACE_HPP

#include <ros/ros.h>

enum class WristControl   { UP, DOWN, DOWN_2_3 };
enum class GripperControl { OPEN, CLOSED };

struct DriveControl
{
   double left;
   double right;
};

struct Action
{
   DriveControl   drive;
   WristControl   wrist;
   GripperControl grip;
};

class RobotInterface
{
private:
   ros::NodeHandle _nh;
   ros::Publisher _drivePublisher;
   ros::Publisher _gripPublisher;
   ros::Publisher _wristPublisher;

   const float WRIST_UP_ANGLE = 0;
   const float WRIST_DOWN_ANGLE = 1.25;
   const float GRIP_OPEN_ANGLE = 3.14159/2;
   const float GRIP_CLOSED_ANGLE = 0;
   
public:
   RobotInterface(std::string name);
   ~RobotInterface();
   void DoAction(Action a);
};

#endif // _ROBOT_INTERFACE_HPP
