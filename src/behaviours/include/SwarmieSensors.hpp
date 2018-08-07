#ifndef _SWARMIE_SENSORS_HPP
#define _SWARMIE_SENSORS_HPP

#include <iostream> // ostream

#include "Tag.hpp"
#include "Point.hpp"
#include "Heading.hpp"
#include "Sensors.hpp"

class SwarmieSensors : public core::Sensors
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
   void SetDeadReckoningPosition(Point p) { _deadReckoningPosition = p; }
   void SetGPSFusedPosition(Point p) { _gpsFusedPosition = p; }
   void SetHeading(Heading h) { _heading = h; }
   void DetectedTag(Tag t);
   void ClearDetections();
   
   double  GetLeftSonar() const { return _leftSonar; }
   double  GetRightSonar() const { return _rightSonar; }
   double  GetCenterSonar() const { return _centerSonar; }
   Point   GetDeadReckoningPosition() const { return _deadReckoningPosition; }
   Point   GetGPSFusedPosition() const { return _gpsFusedPosition; }
   Point   GetPosition() const { return _deadReckoningPosition; }
   Heading GetHeading() const { return _heading; }

   const std::vector<Tag> GetTags() const { return _detections; }
};

#endif // _SWARMIE_SENSORS_HPP