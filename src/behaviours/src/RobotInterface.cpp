#include "RobotInterface.hpp"

#include <std_msgs/Float32.h>
#include <swarmie_msgs/Skid.h>

RobotInterface::RobotInterface(std::string name) :
   _nh()
{
   _gripPublisher  = _nh.advertise<std_msgs::Float32>(name + "/fingerAngle/cmd", 1, true);
   _wristPublisher = _nh.advertise<std_msgs::Float32>(name + "/wristAngle/cmd", 1, true);
   _drivePublisher = _nh.advertise<swarmie_msgs::Skid>(name + "/driveControl/cmd", 1, true);
}

RobotInterface::~RobotInterface() {}

void RobotInterface::DoAction(Action a)
{
   std_msgs::Float32 finger;
   finger.data = (a.grip == GripperControl::OPEN ? GRIP_OPEN_ANGLE : GRIP_CLOSED_ANGLE);
   std_msgs::Float32 wrist;
   if(a.wrist == WristControl::UP)
   {
      wrist.data = WRIST_UP_ANGLE;
   }
   else if(a.wrist == WristControl::DOWN)
   {
      wrist.data = WRIST_DOWN_ANGLE;
   }
   else if(a.wrist == WristControl::DOWN_2_3)
   {
      wrist.data = 0.666666 * WRIST_DOWN_ANGLE;
   }
   swarmie_msgs::Skid skid;
   skid.left = a.drive.left;
   skid.right = a.drive.right;

   _gripPublisher.publish(finger);
   _wristPublisher.publish(wrist);
   _drivePublisher.publish(skid);
}
