#include "test_util.h"
#include "SwarmieInterface.hpp"

bool is_moving(core::Action a)
{
   return (a.GetType() == core::Action::Type::VELOCITY)
      && (a.GetVelocity().GetLinearMagnitude() > 0.01 || a.GetVelocity().GetAngularMagnitude() > 0.01);
}

bool is_moving_forward(core::Action a)
{
   if(a.GetType() != core::Action::Type::VELOCITY)
   {
      return false;
   }
   else
   {
      core::VelocityAction v = a.GetVelocity();
      return v.GetX() > 0.01 && v.GetY() == 0 && v.GetZ() == 0 && v.GetAngularMagnitude() < 0.01;
   }
}

bool is_turning_left(core::Action a)
{
   if(!is_moving(a)) {
      return false;
   }
   
   return a.GetVelocity().GetAngularMagnitude() > 0.01 && a.GetVelocity().GetYaw() > 0;
}

bool is_turning_right(core::Action a)
{
   if(!is_moving(a)) {
      return false;
   }
   
   return a.GetVelocity().GetAngularMagnitude() > 0.01 && a.GetVelocity().GetYaw() < 0;
}

/* Tag positions values measured from the robot camera:
 *
 * _____________
 * |A         B|
 * |           |
 * |           |
 * |           |
 * |D         C|
 * -------------
 *
 * A -> x: -0.202 | y: -0.121 | z: 0.5
 * B -> x: 0.213  | y: -0.116 | z: 0.63
 * C -> x: 0.068  | y: 0.042  | z: 0.212
 * D -> xL -0.069 | y: 0.039  | z: 0.63
 */

Tag tag_top_left(int id)
{
   return Tag(id, -0.202, -0.121, 0.5, POSITIVE_ORIENTATION);
}

Tag tag_top_right(int id)
{
   return Tag(id, 0.213, -0.116, 0.63, POSITIVE_ORIENTATION);
}

Tag tag_bottom_left(int id)
{
   return Tag(id, -0.069, 0.039, 0.63, POSITIVE_ORIENTATION);
}

Tag tag_bottom_right(int id)
{
   return Tag(id, 0.068, 0.042, 0.212, POSITIVE_ORIENTATION);
}

Tag negative_yaw_tag(int id)
{
   return Tag(id, 0.02, 0.0, 0.4, NEGATIVE_ORIENTATION);
}