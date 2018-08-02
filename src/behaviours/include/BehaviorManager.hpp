#ifndef _BEHAVIOR_MANAGER_HPP
#define _BEHAVIOR_MANAGER_HPP

#include <ros/ros.h>

#include "SwarmieInterface.hpp"
#include "SwarmieSensors.hpp"

#include <vector>
#include <memory>

class Behavior
{
protected:
   SwarmieAction _action;

   Behavior* _subsumedBehavior;
   SwarmieAction GetSubsumedAction();
public:
   Behavior();
   virtual ~Behavior();

   SwarmieAction GetAction() const { return _action; }
   virtual void Update(const SwarmieSensors& sensors, const SwarmieAction& ll_action) { _action = ll_action; }
   virtual void Reset() { }

   void Subsumes(Behavior* b) { _subsumedBehavior = b; }
};

class Constant : public Behavior
{
public:
   Constant(SwarmieAction a) { _action = a; }
   ~Constant() {}
   void Update(const SwarmieSensors& sensors, const SwarmieAction& ll_action) override { /* Don't do anything */ }
};

class BehaviorManager
{
public:
   BehaviorManager();
   ~BehaviorManager();

   void RegisterCore(std::shared_pointer<core::RobotInterface> core_interface,
                     std::shared_pointer<core::BehaviorStack> core_behaviors);
   void RegisterMission(std::shared_pointer<mission::RobotInterface> mission_interface,
                        std::shared_pointer<mission::BehaviorStack> mission_behaviors);

private:
   SwarmieSensors* _sensors;
   std::shared_pointer<core::RobotInterface>    _core_interface;
   std::shared_pointer<mission::RobotInterface> _core_interface;
   std::shared_pointer<core::BehaviorStack>     _core_behaviors;
   std::shared_pointer<mission::BehaviorStack>  _mission_behaviors;
};

#endif // _BEHAVIOR_MANAGER_HPP
