#include "SwarmieSensors.hpp"

SwarmieSensors::SwarmieSensors(std::string name) :
   _nh()
{
   _leftSubscriber = _nh.subscribe(name + "/sonarLeft", 1, &SwarmieSensors::LeftSonarHandler, this);
   _rightSubscriber = _nh.subscribe(name + "/sonarRight", 1, &SwarmieSensors::RightSonarHandler, this);
   _centerSubscriber = _nh.subscribe(name + "/sonarCenter", 1, &SwarmieSensors::CenterSonarHandler, this);

   _tagSubscriber = _nh.subscribe(name + "/targets", 1, &SwarmieSensors::TagHandler, this);
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

Tag::Tag(int id, double x, double y, double z, double yaw) :
   _id(id),
   _x(x),
   _y(y),
   _z(z),
   _yaw(yaw)
{}

double Tag::GetAlignment(double offset) const
{
   return _x - offset;
}

double Tag::GetDistance(double offset, double height) const
{
   double distance = hypot(hypot(_x - offset, _y), _z);
   double horizontalDistance;
   if(distance * distance - height * height > 0)
   {
      horizontalDistance = sqrt(distance * distance - height * height);
   }
   else
   {
      // an arbitrary small distance
      horizontalDistance = 0.0000001;
   }
   
   return horizontalDistance;
}
