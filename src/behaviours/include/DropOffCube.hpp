#ifndef _DROP_OFF_CUBE_HPP
#define _DROP_OFF_CUBE_HPP

#include "BehaviorManager.hpp"
#include "PID.hpp"
#include "Timer.hpp"

class DropOffCube : public Behavior
{
private:
   enum State { NotHolding, Holding, Centering, Entering, Droping, Leaving } _state;
   void TagHandler(const SwarmieSensors& sensors);

   /**
    * Transition states based on lower level action.
    */
   void UpdateState(const SwarmieSensors& sensors, const SwarmieAction& ll_action);

   /**
    * Transition states based on timeouts.
    */
   void Timeout();
   void CheckTimeout();

   void Center();
   void Enter();
   void Leave();
   
   int    _numNestTags;
   double _averageAlignment;
   
   Timer* _timer;

   PID _centeringPID;

   // once EXIT_THRESHOLD tags have been seen the rover has "exited"
   // the collection zone.
   const int EXIT_THRESHOLD     = 5;
   const int APPROACH_THRESHOLD = 8;

public:
   DropOffCube(Timer* timer);
   
   ~DropOffCube() {}

   void Update(const SwarmieSensors& sensors, const SwarmieAction& ll_action) override;
};

#endif // _DROP_OFF_CUBE_HPP