#include "PickUpCube.hpp"
#include "RobotInterface.hpp"

PickUpCube::PickUpCube(std::string name, double cameraOffset, double cameraHeight) :
   _cameraHeight(cameraHeight),
   _cameraOffset(cameraOffset),
   _state(NotHolding),
   _allowReset(true),
   _distanceToTarget(OUT_OF_RANGE)
{
   _tagSubscriber = _nh.subscribe(name + "/targets", 1, &PickUpCube::TagHandler, this);
   _pickupTimer = _nh.createTimer(ros::Duration(0), &PickUpCube::PickupTimeout, this, true);
   _pickupTimer.stop();
   _checkTimer  = _nh.createTimer(ros::Duration(4), &PickUpCube::CheckTimeout, this, true);
   _checkTimer.stop();
   _recheckTimer = _nh.createTimer(ros::Duration(30), &PickUpCube::RecheckHandler, this);
   _centerSonar = _nh.subscribe(name + "/sonarCenter", 1, &PickUpCube::SonarHandler, this);
}

void PickUpCube::SetRecheckInterval(double t)
{
   _recheckTimer.stop();
   _recheckTimer.setPeriod(ros::Duration(t), true);
   _recheckTimer.start();
}

void PickUpCube::SonarHandler(const sensor_msgs::Range& message)
{
   _centerRange = message.range;
}

void PickUpCube::CheckTimeout(const ros::TimerEvent& event)
{
   std::cout << "in check timeout" << std::endl;
   // only transition if we meant to check.
   if(_state != Checking && _state != Rechecking) return;
   
   if(_centerRange < 0.12 || _distanceToTarget < 0.14)
   {
      _state = Holding;
      _recheckTimer.start();
   }
   else
   {
      _state = NotHolding;
   }
}

void PickUpCube::RecheckHandler(const ros::TimerEvent& event)
{
   if(_state == Holding)
   {
      _checkTimer.stop();
      _checkTimer.setPeriod(ros::Duration(3), true);
      std::cout << "Holding -> Checking" << std::endl;
      _state = Rechecking;
      _checkTimer.start();
   }
}

void PickUpCube::TagHandler(const apriltags_ros::AprilTagDetectionArray::ConstPtr& message)
{
   _distanceToTarget = OUT_OF_RANGE;

   for(auto tag : message->detections)
   {
      if(tag.id != 0) continue;

      auto p = tag.pose.pose.position;

      switch(_state)
      {
      case NotHolding:
         if(Aligned(p) && Distance(p) < PICKUP_DISTANCE)
         {
            _state = LastInch;
         }
         break;
      case Checking:
         if(p.x - _cameraOffset < 0.01)
         {
            _distanceToTarget = hypot(hypot(p.x, p.y), p.z);
         }
         break;
      default:
         break;
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
      _checkTimer.stop();
      _checkTimer.setPeriod(ros::Duration(3), true);
      std::cout << "Raise -> Checking" << std::endl;
      _state = Checking;
      _checkTimer.start();
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

void PickUpCube::Update()
{
   _action = _llAction;

   switch(_state)
   {
   case LastInch:
      std::cout << "in LastInch" << std::endl;
      _action.drive.left = 40;
      _action.drive.right = 40;
      ResetTimer(0.5);
      break;
   case Grip:
      std::cout << "in Grip" << std::endl;
      _action.drive.left = 0;
      _action.drive.right = 0;
      _action.grip = GripperControl::CLOSED;
      _action.wrist = WristControl::DOWN;
      ResetTimer(1.5);
      break;
   case Raise:
      std::cout << "in Raise" << std::endl;
      _action.drive.left = 0;
      _action.drive.right = 0;
      _action.grip = GripperControl::CLOSED;
      _action.wrist = WristControl::UP;
      ResetTimer(2);
      break;
   case Holding:
      std::cout << "in Holding" << std::endl;
      _action = _subsumedBehavior->GetAction();
      _action.grip = GripperControl::CLOSED;
      _action.wrist = WristControl::DOWN_2_3;
      break;
   case Checking:
      std::cout << "in Checking" << std::endl;
      _action.drive.left = 0;
      _action.drive.right = 0;
   case Rechecking:
      _action.grip = GripperControl::CLOSED;
      _action.wrist = WristControl::UP;
      break;
   default:
      std::cout << "in Wait" << std::endl;
      break;
   }
}
