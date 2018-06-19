#include "ObstacleBehavior.hpp"

#include <math.h>

#define DRIVE_MAX 512

ObstacleBehavior::ObstacleBehavior(const SwarmieSensors* sensors) :
   Behavior(sensors)
{}

void ObstacleBehavior::Update()
{
   _action = _llAction;

   if(_sensors->GetLeftSonar() > 0.6)
      _action.drive.left  = 1/pow(0.6 - _sensors->GetLeftSonar(), 4);
   else
      _action.drive.left = DRIVE_MAX;

   if(_sensors->GetRightSonar() > 0.6)
      _action.drive.right = 1/pow(0.6 - _sensors->GetRightSonar(), 4);
   else
     _action.drive.right = DRIVE_MAX;

   if(_sensors->GetCenterSonar() < 0.8) {
      if(_sensors->GetLeftSonar() < _sensors->GetRightSonar())
      {
         _action.drive.right = -_action.drive.right;
      }
      else
      {
         _action.drive.left = -_action.drive.left;
      }
   }
}
