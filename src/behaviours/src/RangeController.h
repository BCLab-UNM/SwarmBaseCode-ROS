#ifndef RANGECONTROLLER_H
#define RANGECONTROLLER_H

// This class implements behaviour that prevents robots from
// leaving the defined foraging range. The range can be
// defined as a circle, square, or rectangle. 
// The center and dimensions of the allowed foraging range
// are specified by the user.
 
#include "Controller.h"
#include <exception> // For exception handling
#include <string> // For dynamic exception messages

// Define the possible range shapes

// Define an exception to be thrown if the user tries to create
// a RangeShape using invalid dimensions
class RangeShapeInvalidParameterException : public std::exception {
 public:
  RangeShapeInvalidParameterException(std::string msg) {
    this->msg = msg;
  }

  virtual const char* what() const throw()
  {
    std::string message = "Invalid parameter used to define a class derived from RangeShape: " + msg;
    return message.c_str();
  }

 private:
  std::string msg;
};

// RangeShape is the base class for all shapes
// used to define a robots foraging range.
// Derived types are used here to simplify the 
// implementation of the controller. The virtual isInside()
// function should be called on the base class
// which will result in the most derived version
// of isInside() being called using the appropriate
// shape.

class RangeShape {
 public: 
  RangeShape();
  
  virtual bool isInside( Point coords ) = 0;
  Point getCenter();

 protected:
  // All shapes have a center
  Point center;
};

// RangeCircle is a derived type that can calculate
// whether a given set of coordinates are inside a
// circle defining the foraging range for a robot.
class RangeCircle : public RangeShape {

 public:
  RangeCircle( Point center, float radius ); 
  
  bool isInside( Point coords ) override;

 private: 
  float radius = 0.0;
};

// RangeRectangle is a derived type that can calculate
// whether a given set of coordinates are inside a
// rectangle defining the foraging range for a robot.
class RangeRectangle : public RangeShape {

 public:
  RangeRectangle(); // Default constructor
  RangeRectangle( Point center, float width, float height ); 
  
  bool isInside( Point coords ) override;

 protected: 
  float width = 0.0;
  float height = 0.0;
};

// Define exceptions for RangeController
class RangeControllerInvalidParameterException : public std::exception {
 public:
  RangeControllerInvalidParameterException(std::string msg) {
    this->msg = msg;
  }

  virtual const char* what() const throw()
  {
    std::string message = "Invalid parameter used in RangeController: " + msg;
    return message.c_str();
  }

 private:
  std::string msg;
};

// ****************************************
// * This controller causes robots
// * to return to a defined search
// * area whenever they leave it.
// *
// * In other words this controller
// * implements a virtual fence.
// *****************************************
class RangeController : virtual Controller {

 public:
  // Constructors
  RangeController();
  RangeController( float backtrack_distance );
  RangeController(float backtrack_distance, RangeShape* range);
    
  // Required interface for controllers
  void Reset() override;
  Result DoWork() override;
  bool ShouldInterrupt() override;
  bool HasWork() override;

  // Setters
  void setRangeShape( RangeShape* range );
  void setBacktrackDistance( float backtrack_distance );
  void setCurrentLocation( Point current );
  void setEnabled( bool enabled );  
  
  // Destructor
  ~RangeController();

 private:

  // Required by controller interface
  void ProcessData();

  Point distAlongLineSegment(Point start, Point end, float dist);

  RangeShape* range = NULL;
  
  // Distance in meters to move towards the center
  // when a rover leaves the allowed range.
  float backtrack_distance = 1.0; // Defeult to 1.0m 
  Point current_location;
  
  // Whether the range restriction is enabled or not
  bool enabled;

  // Remember whether we are already returning to the allowed forage range
  bool requested_return_to_valid_range = false;

};


#endif // End RANGECONTROLLER_H
