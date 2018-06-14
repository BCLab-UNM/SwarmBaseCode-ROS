#include "BehaviorManager.hpp"

#include <unistd.h>

Behavior::Behavior() :
   _nh()
{
   _llAction.drive.left  = 0.0;
   _llAction.drive.right = 0.0;

   _llAction.wrist = WristControl::DOWN;
   _llAction.grip  = GripperControl::OPEN;
}

Behavior::~Behavior() {}

BehaviorManager::BehaviorManager() :
   _behaviors()
{
   Action do_nothing_action;
   do_nothing_action.drive.left = 0.0;
   do_nothing_action.drive.right = 0.0;
   do_nothing_action.wrist = WristControl::DOWN;
   do_nothing_action.grip = GripperControl::OPEN;

   // Create the lowest level action which lowers and opens the
   // gripper and does not move.
   _base_behavior = new Constant(do_nothing_action);
}

BehaviorManager::~BehaviorManager()
{
   delete _base_behavior;
}

void BehaviorManager::RegisterBehavior(Behavior* b)
{
   _behaviors.push_back(b);
}

Action BehaviorManager::NextAction()
{
   // No need to spin, the main() function will call ros::spin() once
   // everything is set up.
   // ros::spinOnce();
   
   Action nextAction = _base_behavior->GetAction();

   for(int level = 0; level < _behaviors.size(); level++)
   {
      _behaviors[level]->SetLowerLevelAction(nextAction);
      nextAction = _behaviors[level]->GetAction();
   }

   return nextAction;
}

