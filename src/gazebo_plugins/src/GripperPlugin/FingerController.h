#ifndef FINGER_CONTROLLER_H
#define FINGER_CONTROLLER_H

#include <ros/console.h>
#include "PIDController.h"

/**
 * This class implements a finger-specific version of the PIDController.
 *
 * @author Matthew Fricke
 * @author Antonio Griego
 */
class FingerController : public PIDController {

  public:

    FingerController();
    FingerController(std::string name, PIDController::PIDSettings settings);

  private:

    // name of the finger joint this controller manipulates
    // naming convention: "roverName_fingerJointNameFromSDF"
    std::string fingerJointName;

};

#endif /* FINGER_CONTROLLER_H */
