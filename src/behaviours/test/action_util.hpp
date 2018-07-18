#ifndef _ACTION_UTIL_HPP
#define _ACTION_UTIL_HPP

#include "Action.hpp"

bool is_nil(const VelocityAction& vel);
bool angular_only(const VelocityAction& vel);
bool linear_only(const VelocityAction& vel);

#endif // _ACTION_UTIL_HPP
