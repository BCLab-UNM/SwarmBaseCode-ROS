#include "StraightLineBehavior.hpp"
#include "Velocity.hpp"

StraightLineBehavior::StraightLineBehavior() :
   Behavior(nullptr)
{}

void StraightLineBehavior::Update()
{
   _action = _llAction;

   // TODO: guard against waypoint actions
   if(_llAction.GetVelocity().GetAngularMagnitude() < 0.75)
   {
      core::VelocityAction v = _llAction.GetVelocity();
      v.SetLinear(LinearVelocity(0.5));
      _action.SetAction(v);
   }
}
