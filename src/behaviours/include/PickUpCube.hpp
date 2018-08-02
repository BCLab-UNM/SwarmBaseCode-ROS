#ifndef _PICK_UP_CUBE_HPP
#define _PICK_UP_CUBE_HPP

#include "BehaviorManager.hpp"
#include "SwarmieSensors.hpp"
#include "Timer.hpp"

class PickUpCube : public Behavior
{
private:
   enum State { LastInch, Grip, Raise, Holding, Rechecking, Checking, NotHolding } _state;
   
   const double PICKUP_DISTANCE     = 0.3;
   const double ALIGNMENT_THRESHOLD = 0.005;
   const double OUT_OF_RANGE        = 100;

   SwarmieSensors _sensors;

   Timer* _timer;

   bool   _allowReset;
   double _cameraOffset;
   double _cameraHeight;
   double _distanceToTarget;
   double _recheckInterval;

   void ProcessTags();

   void TimerTriggered();

   inline void StartTimer(double i);
   inline void StartCheckTimer();
   inline void StartRecheckTimer();
   inline void ResetPickupTimer(double time);

public:
   PickUpCube(Timer* timer);

   ~PickUpCube() {}

   void Reset() override;   
   void Update(const SwarmieSensors& sensors, const SwarmieAction& ll_action) override;
   
   void SetRecheckInterval(double t);
};

#endif // _PICK_UP_CUBE_HPP
