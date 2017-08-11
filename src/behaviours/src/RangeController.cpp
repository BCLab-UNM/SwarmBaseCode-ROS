#include "RangeController.h"
#include <cmath> // For square root function
#include <iostream>

RangeShape::RangeShape()
{
}

Point RangeShape::getCenter()
{
  return center;
}

RangeCircle::RangeCircle( Point center, float radius )
{
  // Don't allow circles with negative radii
  if ( radius < 0 ) throw RangeShapeInvalidParameterException("(Circle Range)");

  this->center = center;
  this->radius = radius;
}

bool RangeCircle::isInside( Point coords)
{  
  // Return whether the coordinates given are within the
  // radius of the circle. The mathematical expression is just the Euclidean norm.
  return sqrt((center.x-coords.x)*(center.x-coords.x) + (center.y-coords.y)*(center.y-coords.y)) < radius;
}

RangeRectangle::RangeRectangle( Point center, float width, float height )
{
  // Don't allow rectangles with negative sides
  if ( width < 0 || height < 0 ) throw RangeShapeInvalidParameterException("(Rectangle Range)");

  this->center = center;
  this->width = width;
  this->height = height;
}

bool RangeRectangle::isInside( Point coords )
{  
  if ( coords.x < center.x+width/2.0 
       && coords.x > center.x-width/2.0 
       && coords.y < center.y+height/2.0 
       && coords.y > center.y-height/2.0)
    {    
      // Return whether the coordinates given are within the area of the square.
      return true;
    }
  
return false;
}

// Default constructor
RangeController::RangeController()
{
}

RangeController::RangeController( float backtrack_distance )
{
  setBacktrackDistance( backtrack_distance );
}

RangeController::RangeController( float backtrack_distance, RangeShape* range )
{
  setBacktrackDistance( backtrack_distance );
  setRangeShape( range );
}

void RangeController::Reset()
{
  // Leaving empty until we decide on what reset to defaults really means.
}

Result RangeController::DoWork() 
{
 
  Result result;

  // Move the rover a parameterised distance towards the origin 
  // of the rovers coordinate system. To accomplish this we set a waypoint 
  // between the current location and the origin, store it in the result object,
  // and return it to the logic controller.
  
  Point point_in_range = distAlongLineSegment(current_location, range->getCenter(), backtrack_distance);

  result.type = waypoint;
  result.wpts.waypoints.clear();
  result.wpts.waypoints.push_back( point_in_range );
  result.PIDMode = FAST_PID;
  
  cout << "Range controller doing work: moving to (" << point_in_range.x << ", " << point_in_range.y << ")" << endl;  

  return result;
  
}

bool RangeController::ShouldInterrupt() 
{
  // Cause an interrupt if the rover leaves the specified foraging range
  // Note use of shortcircuiting "and"
  bool should_interrupt = false;
  if (enabled 
      && range != NULL
      && !range->isInside(current_location) 
      && !requested_return_to_valid_range)
    {
      cout << "Range controller interrupt" << endl;
      requested_return_to_valid_range = true;
      should_interrupt = true;
    }
  
  return should_interrupt;    
}

bool RangeController::HasWork() 
{
  bool has_work = false;
  if (enabled && range != NULL && !range->isInside(current_location)) 
    {
      cout << "Range controller has work:  current location (" << current_location.x << ", " << current_location.y << ")" << endl;
      has_work = true;
      // Report that there is work to be done if the rover is outside the specified forgaing range.
      // Note use of shortcircuiting "and"
    }
  else
    { 
      requested_return_to_valid_range = false;
      enabled && range != NULL && !range->isInside(current_location);
      has_work = false;
    }

  return has_work;
}

void RangeController::setCurrentLocation( Point current ) 
{
  current_location = current;
}

void RangeController::setBacktrackDistance( float backtrack_distance )
{
  if (backtrack_distance <= 0) throw RangeControllerInvalidParameterException(" The backtrack distance must be positive.");

  this->backtrack_distance = backtrack_distance;
}

// Set the shape of the valid foraging range
void RangeController::setRangeShape( RangeShape* range )
{
  if ( this->range != NULL ) delete this->range; // Clean up memory
  this->range = range;
}

// Given two points, start and end, that define a line segment, L, this function returns a new point
// that is a distance, dist, away from start along L
Point RangeController::distAlongLineSegment(Point start, Point end, float dist)
{
  Point V; // Create a vector at the origin in the same direction 
           // as the line segment between the start and end
  V.x = end.x - start.x;
  V.y = end.y - start.y;

  // Calculate the magnitude of the vector (same as the length of L)
  float V_mag = std::sqrt(V.x*V.x+V.y*V.y);

  Point U; // A unit vector in the direction of V
  U.x = V.x/V_mag;
  U.y = V.y/V_mag;
  
  Point P; // The point we want. The length is the distance requied times the unit vector.
  P.x = dist*U.x + start.x;
  P.y = dist*U.y + start.y;

  return P;
}

void RangeController::setEnabled( bool enabled )
{
  this->enabled = enabled;
} 

void RangeController::ProcessData() 
{
}

RangeController::~RangeController() 
{
  if ( range != NULL ) delete range;
}
