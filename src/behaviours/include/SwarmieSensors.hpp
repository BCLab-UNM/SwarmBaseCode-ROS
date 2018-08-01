#ifndef _SWARMIE_SENSORS_HPP
#define _SWARMIE_SENSORS_HPP

#include <iostream> // ostream

#include "Tag.hpp"
#include "Point.hpp"
#include "Heading.hpp"

class SwarmieSensors
{
private:
   double _leftSonar;
   double _rightSonar;
   double _centerSonar;
   std::vector<Tag> _detections;

   Point   _deadReckoningPosition;
   Point   _gpsFusedPosition;
   Heading _heading;

public:
   SwarmieSensors();
   ~SwarmieSensors() {};

   void SetLeftSonar(double range) { _leftSonar = range; }
   void SetRightSonar(double range) { _rightSonar = range; }
   void SetCenterSonar(double range) { _centerSonar = range; }
   void DetectedTag(Tag t);
   void ClearDetections();
   
   double GetLeftSonar() const { return _leftSonar; }
   double GetRightSonar() const { return _rightSonar; }
   double GetCenterSonar() const { return _centerSonar; }
   const std::vector<Tag> GetTags() const { return _detections; }
};

#endif // _SWARMIE_SENSORS_HPP