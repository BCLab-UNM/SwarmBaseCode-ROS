#ifndef _APPROACH_CUBE_HPP
#define _APPROACH_CUBE_HPP

#include "BehaviorManager.hpp"
#include "PID.hpp"

class ApproachCube : public Behavior
{
private:
   double _distanceToTag;
   double _alignment;

   PID    _approachPID;

   void ProcessTags(const SwarmieSensors& sensors);
public:
   ApproachCube();
   ~ApproachCube() {}

   void Update(const SwarmieSensors& sensors, const SwarmieAction& ll_action) override;
};

#endif // _APPROACH_CUBE_HPP
