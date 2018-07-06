#ifndef _PICK_UP_CUBE_HPP
#define _PICK_UP_CUBE_HPP

#include "BehaviorManager.hpp"
#include "SwarmieSensors.hpp"

template <typename T>
class PickUpCube : public Behavior
{
private:
   enum State { LastInch, Grip, Raise, Holding, Rechecking, Checking, NotHolding } _state;
   
   const double PICKUP_DISTANCE = 0.3;
   const double ALIGNMENT_THRESHOLD = 0.005;
   const double OUT_OF_RANGE    = 100;

   T _pickupTimer;
   T _checkTimer;
   T _recheckTimer;

   bool   _allowReset;
   double _cameraOffset;
   double _cameraHeight;
   double _distanceToTarget;
   double _centerRange;

   void ProcessTags()
   {
      _distanceToTarget = OUT_OF_RANGE;

      for(auto tag : _sensors->GetTags())
      {
         if(tag.GetId() != 0) continue;

         switch(_state)
         {
         case NotHolding:
            if(fabs(tag.Alignment()) < ALIGNMENT_THRESHOLD
               && tag.HorizontalDistance() < PICKUP_DISTANCE)
            {
               _state = LastInch;
            }
            break;
         case Checking:
            if(fabs(tag.Alignment()) < 0.02)
            {
               if(_distanceToTarget > tag.Distance())
                  _distanceToTarget = tag.Distance();
            }
            break;
         default:
            break;
         }
      }
   }

   void PickupTimeout()
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
         _checkTimer.Stop();
         std::cout << "Raise -> Checking" << std::endl;
         _state = Checking;
         _checkTimer.StartOnce();
         break;
      }
   }

   void CheckTimeout()
   {
      std::cout << "in check timeout" << std::endl;
      // only transition if we meant to check.
      if(_state != Checking && _state != Rechecking) return;
   
      if(_sensors->GetCenterSonar() < 0.12 || _distanceToTarget < 0.14)
      {
         _state = Holding;
         _recheckTimer.StartRepeat();
      }
      else
      {
         _state = NotHolding;
      }
   }

   void RecheckHandler()
   {
      if(_state == Holding)
      {
         _checkTimer.Stop();
         std::cout << "Holding -> Checking" << std::endl;
         _state = Rechecking;
         _checkTimer.StartOnce();
      }
   }

   void ResetPickupTimer(double time)
   {
      if(!_allowReset) return;
      _allowReset = false;
      _pickupTimer.Stop();
      _pickupTimer.SetInterval(time);
      _pickupTimer.StartOnce();
   }

public:
   PickUpCube(const SwarmieSensors* sensors) :
      Behavior(sensors),
      _state(NotHolding),
      _allowReset(true),
      _distanceToTarget(OUT_OF_RANGE),
      _pickupTimer([this]() { this->PickupTimeout(); }),
      _checkTimer([this]() { this->CheckTimeout(); }),
      _recheckTimer([this]() { this->RecheckHandler(); })
   {
      _pickupTimer.SetInterval(0);
      _checkTimer.SetInterval(4);
      _recheckTimer.SetInterval(30);
   }

   ~PickUpCube() {}

   void Reset() override
   {
      _state = NotHolding;
      _allowReset = true;
      _checkTimer.Stop();
      _recheckTimer.Stop();
      _pickupTimer.Stop();
      _distanceToTarget = OUT_OF_RANGE;
   }
   
   void Update() override
   {
      ProcessTags();
      _centerRange = _sensors->GetCenterSonar();
      _action = _llAction;

      switch(_state)
      {
      case LastInch:
         std::cout << "in LastInch" << std::endl;
         _action.drive.left = 60;
         _action.drive.right = 60;
         ResetPickupTimer(0.7);
         break;
      case Grip:
         _action.drive.left = 0;
         _action.drive.right = 0;
         _action.grip = GripperControl::CLOSED;
         _action.wrist = WristControl::DOWN;
         ResetPickupTimer(1.5);
         break;
      case Raise:
         _action.drive.left = 0;
         _action.drive.right = 0;
         _action.grip = GripperControl::CLOSED;
         _action.wrist = WristControl::UP;
         ResetPickupTimer(2);
         break;
      case Holding:
         _action = _subsumedBehavior->GetAction();
         _action.grip = GripperControl::CLOSED;
         _action.wrist = WristControl::DOWN_2_3;
         break;
      case Checking:
         _action.drive.left = 0;
         _action.drive.right = 0;
      case Rechecking:
         _action.grip = GripperControl::CLOSED;
         _action.wrist = WristControl::UP;
         break;
      default:
         break;
      }
   }

   void SetRecheckInterval(double t)
   {
      _recheckTimer.Stop();
      _recheckTimer.SetInterval(t);
      _recheckTimer.StartRepeat();
   }

};

#endif // _PICK_UP_CUBE_HPP
