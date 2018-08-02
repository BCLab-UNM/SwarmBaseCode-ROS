#ifndef _ALIGN_TO_CUBE_HPP
#define _ALIGN_TO_CUBE_HPP

#include "BehaviorManager.hpp"
#include "PID.hpp"

class AlignToCube : public Behavior
{
private:
   double _distanceToTag;
   double _linearDistance;

   double _integral;

   PID    _alignPID;
   
   void ProcessTags(const SwarmieSensors& sensors);

public:
   AlignToCube();
   ~AlignToCube() {}

   void Update(const SwarmieSensors& sensors, const SwarmieAction& action) override;
};

#endif // _ALIGN_TO_CUBE_HPP