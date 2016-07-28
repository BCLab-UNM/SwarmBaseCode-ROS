#include "GripperManager.h"

/**
 * Default constructor. The user must explicitly set the PIDController
 * inherited variables. If this is not done, the default behavior is to mark
 * the GripperManager object as uninitialized and to generate an error when a
 * call to getForces() is made.
 */
GripperManager::GripperManager() {
  isInitialized = false;
}

/**
 * This is the constructor that should be used to properly instantiate the
 * GripperManager. The PIDSettings objects are set in the GripperPlugin and
 * then passed along here when the GripperManager is initialized.
 *
 * @param wristPID   Struct with the six PID controller parameters for the
 *                   wrist joint.
 * @param fingerPID  Struct with the six PID controller parameters for both of
 *                   the finger joints.
 */
GripperManager::GripperManager(PIDController::PIDSettings wristPIDSettings,
    PIDController::PIDSettings fingerPIDSettings) {

  isInitialized = true;
  leftFingerController = PIDController(fingerPIDSettings);
  rightFingerController = PIDController(fingerPIDSettings);
  wristController = PIDController(wristPIDSettings);
}

/**
 * This function takes input in the form of two GripperManager::GripperState
 * structs and calculates the required force necessary to move all of the
 * gripper joints from the currentState to the desiredState.
 *
 * <p>The farther away that a desiredState angle is from a currentState angle
 * the more force that will be applied to move the joint. If the currentState
 * and desiredState are identical or extremely close, almost no force (or zero
 * force) will be applied to a given joint.
 *
 * @param desiredState The target angles to move the gripper joints towards.
 * @param currentState The current angles of the gripper joints.
 * @return The forces to apply to each joint to move them from the currentState
 *         to the desiredState.
 */
GripperManager::GripperForces GripperManager::getForces(
    GripperManager::GripperState desiredState,
    GripperManager::GripperState currentState) {
  if(isInitialized == false) {
    ROS_ERROR_STREAM(
      "[Gripper Plugin]: In GripperManager.cpp: getForces(): GripperManager "
      << "was not properly initialized in the GripperPlugin class!"
    );
    exit(1);
  }

  GripperManager::GripperForces updatedForces;

  // Get the new forces to apply from the PID joint controllers:
  float leftFingerForce = leftFingerController.update(
    desiredState.leftFingerAngle, currentState.leftFingerAngle
  );

  float rightFingerForce = rightFingerController.update(
    desiredState.rightFingerAngle, currentState.rightFingerAngle
  );

  float wristForce = wristController.update(
    desiredState.wristAngle, currentState.wristAngle
  );

  // Populate the gripper forces needed to achive the desired state
  updatedForces.leftFingerForce = leftFingerForce;
  updatedForces.rightFingerForce = rightFingerForce;
  updatedForces.wristForce = wristForce;

  return updatedForces;
}
