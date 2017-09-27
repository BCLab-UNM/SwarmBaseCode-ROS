#ifndef TAGPOINT_H
#define TAGPOINT_H

#include <boost/math/quaternion.hpp> // For quaternion

// Stores AprilTag data
struct TagPoint {
  int id;
  float x;
  float y;
  float z;
  ::boost::math::quaternion<float> orientation;
};

#endif // TAGPOINT_H
