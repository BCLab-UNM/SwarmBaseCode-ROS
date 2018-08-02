#ifndef _TAG_HPP
#define _TAG_HPP

#include <boost/math/quaternion.hpp>

class Tag
{
private:
   int _id;
   double _x;
   double _y;
   double _z;
   boost::math::quaternion<double> _orientation;

   static constexpr double CAMERA_HEIGHT = 0.195;
   static constexpr double CAMERA_OFFSET = -0.023;
public:
   const static int NEST_TAG_ID = 256;
   const static int CUBE_TAG_ID = 0;

   Tag(int id, double x, double y, double z, boost::math::quaternion<double> orientation);
   ~Tag() {}
   
   double Alignment() const;
   double Distance() const;
   double HorizontalDistance() const;
   double GetX() const { return _x; }
   double GetY() const { return _y; }
   double GetZ() const { return _z; }
   boost::math::quaternion<double> GetOrientation() const { return _orientation; }
   double GetYaw()   const;
   double GetPitch() const;
   int    GetId()    const { return _id; }
   bool   IsCube()   const;
   bool   IsNest()   const;

   friend std::ostream& operator<<(std::ostream& os, const Tag& tag);
};

#endif // _TAG_HPP
