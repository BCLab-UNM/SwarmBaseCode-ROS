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

   void ProcessTags();
public:
   ApproachCube(const SwarmieSensors* sensors);
   ~ApproachCube() {}

   void Update() override;
};

#endif // _APPROACH_CUBE_HPP
