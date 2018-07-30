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
   SwarmieAction _llAction;
   SwarmieAction _action;
   const SwarmieSensors* _sensors;

   Behavior* _subsumedBehavior;
   SwarmieAction GetSubsumedAction();
public:
   Behavior(const SwarmieSensors* sensors);
   virtual ~Behavior();

   SwarmieAction GetAction() const { return _action; }
   virtual void Update() { _action = _llAction; }
   virtual void Reset() { }

   void SetLowerLevelAction(SwarmieAction a) { _llAction = a; }
   void Subsumes(Behavior* b) { _subsumedBehavior = b; }
};

class Constant : public Behavior
{
public:
   Constant(SwarmieAction a) : Behavior(nullptr) { _action = a; }
   ~Constant() {}
   void Update() override { /* Don't do anything */ }
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
};

#endif // _BEHAVIOR_MANAGER_HPP
