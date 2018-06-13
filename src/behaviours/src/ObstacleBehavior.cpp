#include "ObstacleBehavior.hpp"

#include <math.h>

ObstacleBehavior::ObstacleBehavior(std::string name) :
   _nh(),
   _left(0),
   _right(0),
   _center(0)
{
   _leftSubscriber   = _nh.subscribe(name + "/sonarLeft", 1, &ObstacleBehavior::LeftSonarCallback, this);
   _rightSubscriber  = _nh.subscribe(name + "/sonarRight", 1, &ObstacleBehavior::RightSonarCallback, this);
   _centerSubscriber = _nh.subscribe(name + "/sonarCenter", 1, &ObstacleBehavior::CenterSonarCallback, this);
}

Action ObstacleBehavior::GetAction()
{
   Action reaction = _llAction;

   reaction.drive.left  = 1/pow(0.5 - _left, 4);
   reaction.drive.right = 1/pow(0.5 - _right, 4);

   if(_center < 0.5) {
      if(_left < _right)
      {
         reaction.drive.right = -reaction.drive.right;
      }
      else
      {
         reaction.drive.left = -reaction.drive.left;
      }
   }

   return reaction;
}
