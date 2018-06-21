#include "StraightLineBehavior.hpp"

StraightLineBehavior::StraightLineBehavior() :
   Behavior(nullptr)
{}

void StraightLineBehavior::Update()
{
   _action = _llAction;
   if(_llAction.drive.left < 75 && _llAction.drive.right < 75
      && _llAction.drive.right >= 0 && _llAction.drive.left >= 0)
   {
      _action.drive.left += 60;
      _action.drive.right += 60;
   }
}
