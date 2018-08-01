#ifndef _TEST_UTIL_H
#define _TEST_UTIL_H

#include <boost/math/quaternion.hpp>
#include "Action.hpp"
#include "Tag.hpp"


#define POSITIVE_ORIENTATION boost::math::quaternion<double>(0.988,-0.025,0.014,0.152)
#define NEGATIVE_ORIENTATION boost::math::quaternion<double>(0.104,0.986,-0.124,0.026)

bool is_moving(core::Action a);
bool is_turning_left(core::Action a);
bool is_turning_right(core::Action a);
bool is_moving_forward(core::Action a);

Tag tag_top_left(int id);
Tag tag_bottom_left(int id);
Tag tag_top_right(int id);
Tag tag_bottom_right(int id);
Tag negative_yaw_tag(int id);
Tag tag_aligned_close(int id);

#endif // _TEST_UTIL_H