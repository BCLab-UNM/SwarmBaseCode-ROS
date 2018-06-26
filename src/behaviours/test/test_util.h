#ifndef _TEST_UTIL_H
#define _TEST_UTIL_H

#include <boost/math/quaternion.hpp>
#include "SwarmieInterface.hpp" // Action
#include "SwarmieSensors.hpp"   // Tag

#define DEFAULT_ORIENTATION boost::math::quaternion<double>(1.2, 1.2, 1.2, 2.1)

bool is_moving(Action a);
bool is_turning_left(Action a);
bool is_turning_right(Action a);

Tag tag_top_left(int id);
Tag tag_bottom_left(int id);
Tag tag_top_right(int id);
Tag tag_bottom_right(int id);

#endif // _TEST_UTIL_H