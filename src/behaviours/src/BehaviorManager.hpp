#ifndef _BEHAVIOR_MANAGER_HPP
#define _BEHAVIOR_MANAGER_HPP

#include <ros/ros.h>

#include "RobotInterface.hpp"

#include <vector>

class Behavior
{
protected:
   Action _llAction;
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
   BehaviorManager(double period, std::string name);
   ~BehaviorManager();

   void DeclareBehavior(Behavior* b);
private:
   ros::NodeHandle _nh;
   std::vector<Behavior*> _behaviors;
   ros::Timer _timer;
   Behavior *_base_behavior;
   RobotInterface _robot;

   void DoStuff(const ros::TimerEvent&);
};

#endif // _BEHAVIOR_MANAGER_HPP
