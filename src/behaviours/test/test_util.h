#ifndef _TEST_UTIL_H
#define _TEST_UTIL_H

#include "SwarmieInterface.hpp" // Action

bool is_moving(Action a);
bool is_turning_left(Action a);
bool is_turning_right(Action a);

#endif // _TEST_UTIL_H