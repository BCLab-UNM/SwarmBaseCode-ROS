#ifndef WRIST_CONTROLLER_H
#define WRIST_CONTROLLER_H

#include <ros/console.h>
#include "PIDController.h"

/**
 * This class implements a wrist-specific version of the PIDController.
 *
 * @author Matthew Fricke
 * @author Antonio Griego
 */
class WristController : public PIDController {

  public:

    WristController();
    WristController(std::string name, PIDController::PIDSettings settings);

  private:

    // name of the wrist joint this controller manipulates
    // naming convention: "roverName_wristJointNameFromSDF"
    std::string wristJointName;

};

#endif /* WRIST_CONTROLLER_H */
