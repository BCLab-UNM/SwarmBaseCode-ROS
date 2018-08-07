#ifndef _BEHAVIOR_MANAGER_HPP
#define _BEHAVIOR_MANAGER_HPP

#include <ros/ros.h>

#include "SwarmieInterface.hpp"
#include "SwarmieSensors.hpp"
#include "Sensors.hpp"
#include "RobotInterface.hpp"

#include <vector>
#include <memory> // shared_ptr

namespace core {

   class Behavior
   {
   protected:
      core::Action _action;
      Behavior* _subsumedBehavior;
      core::Action GetSubsumedAction();
   public:
      Behavior() {}
      ~Behavior() {}

      core::Action GetAction() const { return _action; }
      virtual void Update(const core::Sensors& sensors, const core::Action& ll_action) { _action = ll_action; }
      virtual void Reset() {}

      void Subsumes(Behavior* b) { _subsumedBehavior = b; }
   };

   /**
    * The interface to the stack of core behaviors.
    */
   class BehaviorStack
   {
   public:
      virtual void PushBehavior(core::Behavior& _behavior);
      virtual std::shared_ptr<core::Action> NextAction(const core::Sensors& sensors);
   };
}

namespace mission {

   class Behavior
   {
   protected:
      mission::Action _action;
      Behavior* _subsumedBehavior;
      mission::Action GetSubsumedAction();
   public:
      Behavior() {}
      ~Behavior() {}

      mission::Action GetAction() const { return _action; }
      virtual void Update(const core::Sensors& core_sensors, const core::Action& core_action,
                          const mission::Sensors& mission_sensors, const mission::Action& ll_action)
         { _action = ll_action; }
      virtual void Reset() {}

      void Subsumes(Behavior* b) { _subsumedBehavior = b; }
   };

   /**
    * The interface to the stack of mission behaviors.
    */
   class BehaviorStack
   {
   public:
      virtual void PushBehavior(Behavior& _behavior);
      virtual std::shared_ptr<mission::Action> NextAction(const core::Sensors&    core_sensors,
                                                          const mission::Sensors& mission_sensors,
                                                          std::shared_ptr<core::Action> core_action);
   };
}

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


class BehaviorManager
{
public:
   BehaviorManager();
   ~BehaviorManager();

   void RegisterCore(std::shared_ptr<core::RobotInterface> core_interface,
                     std::shared_ptr<core::BehaviorStack> core_behaviors);
   void RegisterMission(std::shared_ptr<mission::RobotInterface> mission_interface,
                        std::shared_ptr<mission::BehaviorStack> mission_behaviors);

private:
   std::shared_ptr<core::RobotInterface>    _core_interface;
   std::shared_ptr<mission::RobotInterface> _mission_interface;
   std::shared_ptr<core::BehaviorStack>     _core_behaviors;
   std::shared_ptr<mission::BehaviorStack>  _mission_behaviors;
};

#endif // _BEHAVIOR_MANAGER_HPP
