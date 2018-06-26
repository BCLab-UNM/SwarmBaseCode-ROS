#include "test_util.h"
#include "SwarmieInterface.hpp"

bool is_moving(Action a)
{
   return (fabs(a.drive.left) >= 35 || fabs(a.drive.right) >= 35);
}

