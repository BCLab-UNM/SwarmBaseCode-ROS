#include "Tag.hpp"

Tag::Tag(int id, double x, double y, double z, boost::math::quaternion<double> orientation) :
   _id(id),
   _x(x),
   _y(y),
   _z(z),
   _orientation(orientation)
{}

bool Tag::IsCube() const
{
   return _id == CUBE_TAG_ID;
}

bool Tag::IsNest() const
{
   return _id == NEST_TAG_ID;
}

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

double Tag::GetPitch() const
{
   double x = _orientation.R_component_1();
   double y = _orientation.R_component_2();
   double z = _orientation.R_component_3();
   double w = _orientation.R_component_4();

   return asin(-2.0*(x*z - w*y));
}

std::ostream& operator<<(std::ostream& os, const Tag& tag)
{
   os << "Tag[" << tag.GetId() << "]{ alignment: " << tag.Alignment()
      << " | position: (" << tag._x << ", " << tag._y << ", " << tag._z << ")"
      << " | Orientation: " << tag._orientation << "}";
}
