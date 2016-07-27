#ifndef GRIPPER_MANAGER_H
#define GRIPPER_MANAGER_H

#include <ros/console.h>
#include "PIDController.h"

/**
 * This class defines the interface between the GripperPlugin class and the PID
 * controllers for each of the three primary joints: wrist, left finger, and
 * right finger. Structs for passing data to and from the PID controllers are
 * also defined here.
 *
 * <p> Subscribers in the GripperPlugin will obtain float values representing
 * radian angles. These values are passed downstream through this class in
 * order to calculate the forces (in Newtons) that need to be applied in order
 * to move the joints to the desired angles.
 *
 * @author Matthew Fricke
 * @author Antonio Griego
 * @see    WristController
 * @see    FingerController
 * @see    PIDController
 */
class GripperManager {

 public:

  /**
   * This is the input data type for the GripperManager. Input will typically
   * involve two of these structs: current angle and desired angle.
   */
  struct GripperState {
    float wristAngle;
    float leftFingerAngle;
    float rightFingerAngle;
  };

  /**
   * This is the output data type for the GripperManager. The floating point
   * values contained here represents force in Newtons.
   */
  struct GripperForces {
    float wristForce;
    float leftFingerForce;
    float rightFingerForce;
  };

  // constructors
  GripperManager();
  GripperManager(PIDController::PIDSettings wristPIDSettings,
    PIDController::PIDSettings fingerPIDSettings);

  // PID controller interface function
  GripperForces getForces(GripperState desiredState, GripperState currentState);

 private:

  // PID controllers for each joint of the gripper
  PIDController wristController;
  PIDController leftFingerController;
  PIDController rightFingerController;

  // gripper manager status flags
  bool isInitialized;

};

#endif /* GRIPPER_MANAGER_H */
