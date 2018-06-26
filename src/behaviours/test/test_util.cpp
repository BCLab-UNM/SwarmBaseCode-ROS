#include "test_util.h"
#include "SwarmieInterface.hpp"

bool is_moving(Action a)
{
   return (fabs(a.drive.left) >= 35 || fabs(a.drive.right) >= 35);
}

bool is_turning_left(Action a)
{
   if(!is_moving(a)) {
      return false;
   }
   
   // if one wheel is negative then it must be the left wheel.
   // otherwise the right wheel must be greater than the left.
   if(a.drive.left < 0)
   {
      return a.drive.right >= 0;
   }

   // lets say we are turning if right is at least 15% greater than left.
   return a.drive.right > a.drive.left*0.85;
}

bool is_turning_right(Action a)
{
   if(!is_moving(a)) {
      return false;
   }
   
   if(a.drive.right < 0)
   {
      return a.drive.left >= 0;
   }

   // lets say we are turning if right is at least 15% greater than left.
   return a.drive.left > a.drive.right*0.85;   
}



