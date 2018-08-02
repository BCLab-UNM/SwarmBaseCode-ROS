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

   void RegisterBehavior(Behavior* b);
   SwarmieAction NextAction();

private:
   std::vector<Behavior*> _behaviors;
   Behavior *_base_behavior;
   SwarmieSensors* _sensors;
};

#endif // _BEHAVIOR_MANAGER_HPP
