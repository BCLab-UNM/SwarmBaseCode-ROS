#include "SwarmieInterface.hpp"

#include <std_msgs/Float32.h>
#include <swarmie_msgs/Skid.h>

SwarmieInterface::SwarmieInterface(std::string name) :
   _nh()
{
   // Actuator publishers
   _gripPublisher  = _nh.advertise<std_msgs::Float32>(name + "/fingerAngle/cmd", 1, true);
   _wristPublisher = _nh.advertise<std_msgs::Float32>(name + "/wristAngle/cmd", 1, true);
   _drivePublisher = _nh.advertise<swarmie_msgs::Skid>(name + "/driveControl/cmd", 1, true);

   // Sensor subscribers
   _leftSubscriber = _nh.subscribe(name + "/sonarLeft", 1, &SwarmieInterface::LeftSonarHandler, this);
   _rightSubscriber = _nh.subscribe(name + "/sonarRight", 1, &SwarmieInterface::RightSonarHandler, this);
   _centerSubscriber = _nh.subscribe(name + "/sonarCenter", 1, &SwarmieInterface::CenterSonarHandler, this);
   _tagSubscriber = _nh.subscribe(name + "/targets", 1, &SwarmieInterface::TagHandler, this);
}

void SwarmieInterface::TagHandler(const apriltags_ros::AprilTagDetectionArray::ConstPtr& message)
{
   _sensors.ClearDetections();
   for(auto tag : message->detections)
   {
      auto p = tag.pose.pose.position;
      auto o = tag.pose.pose.orientation;
      // TODO: get yaw.
      Tag t(tag.id, p.x, p.y, p.z, boost::math::quaternion<double>(o.x, o.y, o.z, o.w));
      _sensors.DetectedTag(t);
   }
}

void SwarmieInterface::LeftSonarHandler(const sensor_msgs::Range& range)
{
   _sensors.SetLeftSonar(range.range);
}

void SwarmieInterface::RightSonarHandler(const sensor_msgs::Range& range)
{
   _sensors.SetRightSonar(range.range);
}

void SwarmieInterface::CenterSonarHandler(const sensor_msgs::Range& range)
{
   _sensors.SetCenterSonar(range.range);
}

void SwarmieInterface::DoAction(Action a)
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
