#ifndef _BEHAVIOR_MANAGER_HPP
#define _BEHAVIOR_MANAGER_HPP

#include <ros/ros.h>

#include "RobotInterface.hpp"

#include <vector>

class Behavior
{
protected:
   Action _llAction;
   ros::NodeHandle _nh;
public:
   Behavior();
   virtual ~Behavior();
   
   virtual Action GetAction() { return _llAction; };
   void SetLowerLevelAction(Action a) { _llAction = a; }
};

class Constant : public Behavior
{
private:
   Action _constAction;
public:
   Constant(Action a) { _constAction = a; }
   ~Constant() {}
   Action GetAction() override { return _constAction; }
};

class BehaviorManager
{
public:
   BehaviorManager();
   ~BehaviorManager();

   void RegisterBehavior(Behavior* b);
   Action NextAction();

private:
   std::vector<Behavior*> _behaviors;
   Behavior *_base_behavior;
};

#endif // _BEHAVIOR_MANAGER_HPP
