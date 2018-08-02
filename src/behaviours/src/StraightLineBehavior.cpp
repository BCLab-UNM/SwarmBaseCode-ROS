#include "StraightLineBehavior.hpp"
#include "Velocity.hpp"

StraightLineBehavior::StraightLineBehavior()
{}

void StraightLineBehavior::Update(const SwarmieSensors& sensors, const SwarmieAction& ll_action)
{
   _action = ll_action;

   // TODO: guard against waypoint actions
   if(ll_action.GetVelocity().GetAngularMagnitude() < 0.75)
   {
      core::VelocityAction v = ll_action.GetVelocity();
      v.SetLinear(LinearVelocity(0.5));
      _action.SetAction(v);
   }
}
