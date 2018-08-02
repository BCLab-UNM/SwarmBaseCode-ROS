#include "BehaviorManager.hpp"

#include <unistd.h>

Behavior::Behavior() {}

Behavior::~Behavior() {}

BehaviorManager::BehaviorManager() :
   _behaviors()
{
   SwarmieAction do_nothing_action;
   do_nothing_action.SetWrist(WristControl::DOWN);
   do_nothing_action.SetGrip(GripperControl::OPEN);

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

SwarmieAction BehaviorManager::NextAction()
{
   // No need to spin, the main() function will call ros::spin() once
   // everything is set up.
   // ros::spinOnce();
   
   SwarmieAction nextAction = _base_behavior->GetAction();

   for(int level = 0; level < _behaviors.size(); level++)
   {
      _behaviors[level]->Update(*_sensors, nextAction);
      nextAction = _behaviors[level]->GetAction();
   }

   return nextAction;
}

