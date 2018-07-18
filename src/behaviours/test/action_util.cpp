#include "action_util.hpp"

bool is_nil(const VelocityAction& vel)
{
   return vel.GetAngularMagnitude() == 0 &&  vel.GetLinearMagnitude() == 0;
}

bool angular_only(const VelocityAction& vel)
{
   return vel.GetLinearMagnitude() == 0 && vel.GetAngularMagnitude() > 0;
}

bool linear_only(const VelocityAction& vel)
{
   return vel.GetLinearMagnitude() > 0 && vel.GetAngularMagnitude() == 0;
}

