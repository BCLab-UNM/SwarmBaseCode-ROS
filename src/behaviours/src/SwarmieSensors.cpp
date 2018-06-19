#include "SwarmieSensors.hpp"
#include <boost/math/quaternion.hpp>

SwarmieSensors::SwarmieSensors(std::string name) :
   _nh()
{
   _leftSubscriber = _nh.subscribe(name + "/sonarLeft", 1, &SwarmieSensors::LeftSonarHandler, this);
   _rightSubscriber = _nh.subscribe(name + "/sonarRight", 1, &SwarmieSensors::RightSonarHandler, this);
   _centerSubscriber = _nh.subscribe(name + "/sonarCenter", 1, &SwarmieSensors::CenterSonarHandler, this);

   _tagSubscriber = _nh.subscribe(name + "/targets", 1, &SwarmieSensors::TagHandler, this);
}

void SwarmieSensors::TagHandler(const apriltags_ros::AprilTagDetectionArray::ConstPtr& message)
{
   _detections.clear();
   for(auto tag : message->detections)
   {
      auto p = tag.pose.pose.position;
      auto o = tag.pose.pose.orientation;
      // TODO: get yaw.
      Tag t(tag.id, p.x, p.y, p.z, boost::math::quaternion<double>(o.x, o.y, o.z, o.w));
      _detections.push_back(t);
   }
}

void SwarmieSensors::LeftSonarHandler(const sensor_msgs::Range& range)
{
   _leftSonar = range.range;
}

void SwarmieSensors::RightSonarHandler(const sensor_msgs::Range& range)
{
   _rightSonar = range.range;
}

void SwarmieSensors::CenterSonarHandler(const sensor_msgs::Range& range)
{
   _centerSonar = range.range;
}

Tag::Tag(int id, double x, double y, double z, boost::math::quaternion<double> orientation) :
   _id(id),
   _x(x),
   _y(y),
   _z(z),
   _orientation(orientation)
{}

double Tag::Alignment() const
{
   return _x - CAMERA_OFFSET;
}

double Tag::HorizontalDistance() const
{
   double distance = hypot(hypot(_x - CAMERA_OFFSET, _y), _z);
   double horizontalDistance;
   if(distance * distance - CAMERA_HEIGHT * CAMERA_HEIGHT > 0)
   {
      horizontalDistance = sqrt(distance * distance - CAMERA_HEIGHT * CAMERA_HEIGHT);
   }
   else
   {
      // an arbitrary small distance
      horizontalDistance = 0.0000001;
   }
   
   return horizontalDistance;
}

double Tag::Distance() const
{ 
   return hypot(hypot(_x - CAMERA_OFFSET, _y), _z);
}

double Tag::GetYaw() const
{
   double x = _orientation.R_component_1();
   double y = _orientation.R_component_2();
   double z = _orientation.R_component_3();
   double w = _orientation.R_component_4();

   return atan2(2.0*(y*z + w*x), w*w - x*x - y*y + z*z);
}
