#ifndef _DROP_OFF_CUBE_HPP
#define _DROP_OFF_CUBE_HPP

#include "BehaviorManager.hpp"
#include "PID.hpp"

template <typename T>
class DropOffCube : public Behavior
{
private:
   enum State { NotHolding, Holding, Centering, Entering, Leaving } _state;
   void TagHandler()
   {
      _numNestTags = 0;
      _averageAlignment = 0;

      for(auto tag : _sensors->GetTags())
      {
         if(!tag.IsNest()) continue;
         _averageAlignment += tag.GetAlignment();
         _numNestTags++;
      }

      if(_numNestTags > 0)
      {
         _averageAlignment /= _numNestTags;
      }
   }

   /**
    * Transition states based on lower level action.
    */
   void UpdateState()
   {
      switch(_state)
      {
      case NotHolding:
         if(_llAction.grip == GripperControl::CLOSED) {
            _state = Holding;
         }
         break;
      case Holding:
         if(_llAction.grip == GripperControl::OPEN) {
            _state = NotHolding;
         }
         else if(_numNestTags > 0)
         {
            _state = Centering;
            _centeringPID.Reset();
         }
         break;
      case Centering:
         if(_numNestTags == 0)
         {
            // TODO: Don't consider target lost until it is not seen
            // for X ticks
            _state = Holding;
         }
         else if(fabs(_averageAlignment) < 0.02)
         {
            _state = Entering;
            _timer.SetInterval(2.0);
            _timer.StartOnce();
         }
         break;
      }
   }

   /**
    * Transition states based on timeouts.
    */
   void Timeout()
   {
      switch(_state)
      {
      case Entering:
         _state = Droping;
         _timer.SetInterval(1.0);
         _timer.StartOnce();
         break;
      case Droping:
         _state = Leaving:
         _timer.SetInterval(3.0);
         _timer.StartOnce();
         break;
      case Leaving:
         _subsumedBehavior->Reset();
         _state = NotHolding;
         break;
      }
   }

   void Center()
   {
      _centeringPID.Update(_averageAlignment);
      double control = _centeringPID.GetControlOutput();
      _action.drive.left  = 40 * control;
      _action.drive.right = -40 * control;
   }

   void Enter()
   {
      _action.drive.left = 35;
      _action.drive.right = 35;
   }

   void Leave()
   {
      _action.drive.left = -35;
      _action.drive.right = -35;
   }
   
   int    _numNestTags;
   double _averageAlignment;
   
   T _timer;

   PID _centeringPID;

   // once EXIT_THRESHOLD tags have been seen the rover has "exited"
   // the collection zone.
   const int EXIT_THRESHOLD = 5;
   const int APPROACH_THRESHOLD = 8;

public:
   DropOffCube(const SwarmieSensors *sensors) :
      Behavior(sensors),
      _state(NotHolding),
      _timer([this]() { this->Timeout(); }),
      _centeringPID(0.5, 0.2, 0.5) // XXX: Not tuned
   {}
   
   ~DropOffCube() {}

   void Update() override
   {
      TagHandler();
      UpdateState();

      switch(_state)
      {
      case Centering:
         Center();
         break;
      case Entering:
         Enter();
         break;
      case Leaving:
         Leave();
         break;
      case Holding:
      case NotHolding:
         _action = _llAction;
         break;
      }
   }
};

#endif // _DROP_OFF_CUBE_HPP