#include "GripperManager.h"

GripperManager::GripperManager() {
}

GripperManager::~GripperManager() {
}

GripperForces GripperManager::getForces( GripperState desState, GripperState curState ) {
  GripperForces updatedForces;

  // Get the new forces to apply from the PID joint controllers
  float leftFingerForce = leftFingerController.update( desState.leftFingerAngle, curState.leftFingerAngle);
  float rightFingerForce = rightFingerController.update( desState.rightFingerAngle, curState.rightFingerAngle);
  float wristForce = wristController.update( desState.wristAngle, curState.wristAngle);

  // Populate the gripper forces needed to achive the desired state
  updatedForces.leftFingerForce = leftFingerForce;
  updatedForces.rightFingerForce = rightFingerForce;
  updatedForces.wristForce = wristForce;

  return updatedForces;
}
