#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "Result.h"

/*
 * This class is meant to serve as a template for all Controllers,
 * including new Controllers defined by each team.
 *
 * It also exists so that Controllers have a generic interface to
 * use for processing interrupts in LogicController.
 */
class Controller {
public:
  Controller() {}

  // Resets internal state to defaults
  virtual void reset() = 0;

  // Determines what action should be taken based on current
  // internal state and data
  virtual Result doWork() = 0;

  // Returns whether or not an interrupt must be thrown
  virtual bool shouldInterrupt() = 0;

  // Returns whether or not a controller should be polled for a Result
  virtual bool hasWork() = 0;

  ~Controller() {}

protected:

  // Looks at external data and determines if an interrupt must be thrown
  // or if the controller should be polled
  virtual void processData() = 0;
};

#endif // CONTROLLER_H
