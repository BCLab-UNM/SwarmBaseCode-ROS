#include "StraightLineBehavior.hpp"

StraightLineBehavior::StraightLineBehavior(std::string name) :
   _nh(),
   _heading_set(false)
{
   _odomSubscriber = _nh.subscribe(name + "/odom/filtered", 1, &StraightLineBehavior::OdometryHandler, this);
}

void StraightLineBehavior::OdometryHandler(const nav_msgs::Odometry::ConstPtr& message)
{
   // The idea is that this can be a PID to maintain a straigt line. For now just drive.
}

Action StraightLineBehavior::GetAction()
{
   Action reaction = _llAction;
   if(_llAction.drive.left < 75 && _llAction.drive.right < 75
      && _llAction.drive.right >= 0 && _llAction.drive.left >= 0)
   {
      reaction.drive.left += 100;
      reaction.drive.right += 100;
   }
   return reaction;
}
