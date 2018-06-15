#include "PickUpCube.hpp"
#include "RobotInterface.hpp"

PickUpCube::PickUpCube(std::string name, double cameraOffset, double cameraHeight) :
   _cameraHeight(cameraHeight),
   _cameraOffset(cameraOffset),
   _state(Wait),
   _allowReset(true)
{
   _tagSubscriber = _nh.subscribe(name + "/targets", 1, &PickUpCube::TagHandler, this);
   _pickupTimer = _nh.createTimer(ros::Duration(0), &PickUpCube::PickupTimeout, this, true);
}

void PickUpCube::TagHandler(const apriltags_ros::AprilTagDetectionArray::ConstPtr& message)
{
   for(auto tag : message->detections)
   {
      if(tag.id != 0) continue;

      if(Aligned(tag.pose.pose.position)
         && Distance(tag.pose.pose.position) < PICKUP_DISTANCE)
      {
         _state = LastInch;
      }
   }
}

void PickUpCube::PickupTimeout(const ros::TimerEvent& event)
{
   _allowReset = true;
   switch(_state)
   {
   case LastInch:
      _state = Grip;
      break;
   case Grip:
      _state = Raise;
      break;
   case Raise:
   default:
      _state = Wait;
      break;
   }
}

bool PickUpCube::Aligned(const geometry_msgs::Point p)
{
   std::cout << "alignment " << fabs(p.x - _cameraOffset) << std::endl;
   return (fabs(p.x - _cameraOffset) < 0.005);
}

double PickUpCube::Distance(const geometry_msgs::Point p)
{
   double distanceFromCamera;
   distanceFromCamera = hypot(hypot(p.x, p.y), p.z);
   distanceFromCamera *= distanceFromCamera;
   double ch = _cameraHeight*_cameraHeight;

   if((distanceFromCamera - ch) > 0)
      return sqrt(distanceFromCamera - ch);
   else
      return 0.00001; // arbitrary small distance
}

void PickUpCube::ResetTimer(double duration)
{
   if(!_allowReset) return;
   _allowReset = false;
   _pickupTimer.stop();
   _pickupTimer.setPeriod(ros::Duration(duration));
   _pickupTimer.start();
}

Action PickUpCube::GetAction()
{
   Action reaction = _llAction;

   switch(_state)
   {
   case LastInch:
      reaction.drive.left = 120;
      reaction.drive.right = 120;
      ResetTimer(0.2);         
      break;
   case Grip:
      reaction.drive.left = 0;
      reaction.drive.right = 0;
      reaction.grip = GripperControl::CLOSED;
      reaction.wrist = WristControl::DOWN;
      ResetTimer(1.5);
      break;
   case Raise:
      reaction.drive.left = 0;
      reaction.drive.right = 0;
      reaction.grip = GripperControl::CLOSED;
      reaction.wrist = WristControl::UP;
      ResetTimer(3);
      break;
   default:
      break;
   }

   return reaction;
}
