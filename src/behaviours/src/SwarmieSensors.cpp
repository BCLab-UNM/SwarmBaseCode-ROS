#include "SwarmieSensors.hpp"
#include <boost/math/quaternion.hpp>

SwarmieSensors::SwarmieSensors()
{}

void SwarmieSensors::DetectedTag(Tag t)
{
   _detections.push_back(t);
}

void SwarmieSensors::ClearDetections()
{
   _detections.clear();
}