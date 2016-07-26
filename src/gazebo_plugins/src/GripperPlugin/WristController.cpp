#include "WristController.h"

/**
 * This constructor calls the equivalent constructor from the base
 * PIDController class.
 */
WristController::WristController() :
  PIDController()
{
  wristJointName = "invalid_joint";
}

/**
 * This constructor calls the equivalent constructor from the base
 * PIDController class.
 */
WristController::WristController(std::string name, PIDController::PIDSettings settings) :
  PIDController(settings)
{
  wristJointName = name;
}
