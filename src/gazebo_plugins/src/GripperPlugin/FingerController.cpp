#include "FingerController.h"

/**
 * This constructor calls the equivalent constructor from the base
 * PIDController class.
 */
FingerController::FingerController() :
  PIDController()
{
  fingerJointName = "invalid_joint";
}

/**
 * This constructor calls the equivalent constructor from the base
 * PIDController class.
 */
FingerController::FingerController(std::string name, PIDController::PIDSettings settings) :
  PIDController(settings)
{
  fingerJointName = name;
}
