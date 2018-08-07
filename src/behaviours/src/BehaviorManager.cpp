#include "BehaviorManager.hpp"

#include <unistd.h>

Behavior::Behavior() {}

Behavior::~Behavior() {}

BehaviorManager::BehaviorManager()
{}

BehaviorManager::~BehaviorManager()
{}

void BehaviorManager::RegisterCore(std::shared_ptr<core::RobotInterface> core_interface,
                                   std::shared_ptr<core::BehaviorStack>  core_behaviors)
{
   _core_interface = core_interface;
   _core_behaviors = core_behaviors;
}

void BehaviorManager::RegisterMission(std::shared_ptr<mission::RobotInterface> mission_interface,
                                      std::shared_ptr<mission::BehaviorStack>  mission_behaviors)
{
   _mission_interface = mission_interface;
   _mission_behaviors = mission_behaviors;
}

#if 0
SwarmieAction BehaviorManager::NextAction(const SwarmieSensors& sensors)
{
   SwarmieAction nextAction = _base_behavior->GetAction();

   for(int level = 0; level < _behaviors.size(); level++)
   {
      _behaviors[level]->Update(sensors, nextAction);
      nextAction = _behaviors[level]->GetAction();
   }

   return nextAction;
}
#endif