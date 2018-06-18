#include "ObstacleBehavior.hpp"

#include <math.h>

#define DRIVE_MAX 512

ObstacleBehavior::ObstacleBehavior(std::string name) :
   _left(0),
   _right(0),
   _center(0)
{
   _leftSubscriber   = _nh.subscribe(name + "/sonarLeft", 1, &ObstacleBehavior::LeftSonarCallback, this);
   _rightSubscriber  = _nh.subscribe(name + "/sonarRight", 1, &ObstacleBehavior::RightSonarCallback, this);
   _centerSubscriber = _nh.subscribe(name + "/sonarCenter", 1, &ObstacleBehavior::CenterSonarCallback, this);
}

void ObstacleBehavior::Update()
{
   _action = _llAction;

   if(_left > 0.6)
     _action.drive.left  = 1/pow(0.6 - _left, 4);
   else
     _action.drive.left = DRIVE_MAX;

   if(_right > 0.6)
     _action.drive.right = 1/pow(0.6 - _right, 4);
   else
     _action.drive.right = DRIVE_MAX;

   if(_center < 0.8) {
      if(_left < _right)
      {
         _action.drive.right = -_action.drive.right;
      }
      else
      {
         _action.drive.left = -_action.drive.left;
      }
   }
}
