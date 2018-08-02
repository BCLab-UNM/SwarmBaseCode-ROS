#include "SwarmieInterface.hpp"

#include <std_msgs/Float32.h>
#include <swarmie_msgs/Skid.h>
#include <tf/tf.h> // for dealing with orientation

void SwarmieAction::SetWrist(WristControl w)
{
   _wrist = w;
}

void SwarmieAction::SetGrip(GripperControl g)
{
   _grip = g;
}

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
   _odomSubscriber = _nh.subscribe(name + "/odom/filtered", 1, &SwarmieInterface::OdometryHandler, this);
   _gpsFusedSubscriber = _nh.subscribe(name + "/odom/ekf", 1, &SwarmieInterface::GPSFusedHandler, this);
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

void SwarmieInterface::OdometryHandler(const nav_msgs::Odometry::ConstPtr& odom)
{
   Point p(odom->pose.pose.position.x,
           odom->pose.pose.position.y,
           odom->pose.pose.position.z);
   _sensors.SetDeadReckoningPosition(p);
   tf::Quaternion q(odom->pose.pose.orientation.x,
                    odom->pose.pose.orientation.y,
                    odom->pose.pose.orientation.z,
                    odom->pose.pose.orientation.w);
   tf::Matrix3x3 m(q);
   double roll, pitch, yaw;
   m.getRPY(roll, pitch, yaw);
   _sensors.SetHeading(yaw);
}

void SwarmieInterface::GPSFusedHandler(const nav_msgs::Odometry::ConstPtr& odom)
{
   Point p(odom->pose.pose.position.x,
           odom->pose.pose.position.y,
           odom->pose.pose.position.z);
   _sensors.SetGPSFusedPosition(p);
}

void SwarmieInterface::DoAction(const SwarmieAction& a)
{
   std_msgs::Float32 finger;
   finger.data = (a.GripperCommand() == GripperControl::OPEN ? GRIP_OPEN_ANGLE : GRIP_CLOSED_ANGLE);
   std_msgs::Float32 wrist;
   if(a.WristCommand() == WristControl::UP)
   {
      wrist.data = WRIST_UP_ANGLE;
   }
   else if(a.WristCommand() == WristControl::DOWN)
   {
      wrist.data = WRIST_DOWN_ANGLE;
   }
   else if(a.WristCommand() == WristControl::DOWN_2_3)
   {
      wrist.data = 0.666666 * WRIST_DOWN_ANGLE;
   }

   swarmie_msgs::Skid skid;
   // TODO: Waypoint Action
   if(a.GetType() == core::Action::Type::VELOCITY)
   {
      core::VelocityAction v = a.GetVelocity();
      // Swarmie only uses yaw and x
      skid.left  = v.GetX()*255 - v.GetYaw()*255;
      skid.right = v.GetX()*255 + v.GetYaw()*255;
   }
   else
   {
      skid.left = 0;
      skid.right = 0;
   }

   _gripPublisher.publish(finger);
   _wristPublisher.publish(wrist);
   _drivePublisher.publish(skid);
}
