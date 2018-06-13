#include "BehaviorManager.hpp"

#include <unistd.h>

Behavior::Behavior()
{
   _llAction.drive.left  = 0.0;
   _llAction.drive.right = 0.0;

   _llAction.wrist = WristControl::DOWN;
   _llAction.grip  = GripperControl::OPEN;
}

Behavior::~Behavior() {}

BehaviorManager::BehaviorManager(double period, std::string name) :
   _nh(),
   _behaviors(),
   _robot(name)
{
   Action do_nothing_action;
   do_nothing_action.drive.left = 0.0;
   do_nothing_action.drive.right = 0.0;
   do_nothing_action.wrist = WristControl::DOWN;
   do_nothing_action.grip = GripperControl::OPEN;

   // Create the lowest level action which lowers and opens the
   // gripper and does not move.
   _base_behavior = new Constant(do_nothing_action);

   _timer = _nh.createTimer(ros::Duration(period), &BehaviorManager::DoStuff, this);
}

BehaviorManager::~BehaviorManager()
{
   delete _base_behavior;
}

void BehaviorManager::DeclareBehavior(Behavior* b)
{
   _behaviors.push_back(b);
}

void BehaviorManager::DoStuff(const ros::TimerEvent& event)
{
   // No need to spin, the main() function will call ros::spin() once
   // everything is set up.
   // ros::spinOnce();
   
   Action nextAction = _base_behavior->GetAction();

   std::cout << "timer triggered" << std::endl;
   std::cout << "numlayers: " << _behaviors.size() << std::endl;
   for(int level = 0; level < _behaviors.size(); level++)
   {
      _behaviors[level]->SetLowerLevelAction(nextAction);
      nextAction = _behaviors[level]->GetAction();
   }

   _robot.DoAction(nextAction);
}

