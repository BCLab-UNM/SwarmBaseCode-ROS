#include "LogicController.h"

LogicController::LogicController() {

  logicState = LOGIC_STATE_INTERRUPT;
  processState = PROCCESS_STATE_SEARCHING;

  ProcessData();

  control_queue = priority_queue<PrioritizedController>();

}

LogicController::~LogicController() {}

void LogicController::Reset() {

  std::cout << "LogicController.Reset()" << std::endl;
  logicState = LOGIC_STATE_INTERRUPT;
  processState = PROCCESS_STATE_SEARCHING;

  ProcessData();

  control_queue = priority_queue<PrioritizedController>();
}

//******************************************************************************
// This function is called every 1/10th of a second by the ROSAdapter
// The logical flow if the behaviours is controlled here by using an interrupt,
// haswork, and priority queue system.
Result LogicController::DoWork()
{
  Result result;

  // First, a loop runs through all the controllers who have a priority of 0 or
  // above with the largest number being most important. A priority of less than
  // 0 is an ignored controller (we will use -1 as the standard for an ignored
  // controller). If any controller needs an interrupt, the logic state is
  // changed to interrupt
  for(PrioritizedController cntrlr : prioritizedControllers)
  {
    if(cntrlr.controller->ShouldInterrupt() && cntrlr.priority >= 0)
    {
      logicState = LOGIC_STATE_INTERRUPT;
      // Do not break out of the for loop! All shouldInterupts may need calling
      // in order to properly pre-proccess data.
    }
  }

  switch(logicState) {

  // ***************************************************************************
  // BEGIN LOGIC_STATE_INTERUPT
  // ***************************************************************************

  // Enter this state when an interrupt has been thrown or there are no pending
  // control_queue.top().actions.
  case LOGIC_STATE_INTERRUPT: {
    // Reset the control queue
    control_queue = priority_queue<PrioritizedController>();

    // Check what controllers have work to do.. Every controller, where
    // HasWork() == true, will be added to the priority queue.
    for(PrioritizedController cntrlr : prioritizedControllers) {
      if(cntrlr.controller->HasWork()) {
        if (cntrlr.priority < 0) {
          continue;
        }
        else {
          control_queue.push(cntrlr);
        }
      }
    }

    // If no controlers have work, report this to ROS Adapter and do nothing.
    if(control_queue.empty()) {
      result.type = behavior;
      result.b = wait;
      break;
    }
    else {
      // Default result state if someone has work. This safe gaurds against
      // faulty result types
      result.b = noChange;
    }

    // Take the top member of the priority queue and run its do work function.
    result = control_queue.top().controller->DoWork();

    // Analyze the result that was returned and do state changes accordingly.
    // Behavior types are used to indicate behavior changes.
    if(result.type == behavior) {

      // Ask for an external reset so the state of the controller is preserved
      // until after it has returned a result and gotten a chance to communicate
      // with other controllers.
      if (result.reset) {
        controllerInterconnect(); // Allow controller to communicate state data before it is reset.
        control_queue.top().controller->Reset();
      }

      // Ask for the procces state to change to the next state or loop around to the begining.
      //  enum ProcessState {
      //    _FIRST = 0,
      //    PROCCESS_STATE_SEARCHING = 0,
      //    PROCCESS_STATE_TARGET_PICKEDUP,
      //    PROCCESS_STATE_DROP_OFF,
      //    _LAST,
      //    PROCESS_STATE_MANUAL // robot is under manual control
      //  };
      if(result.b == nextProcess) {
        if (processState == _LAST - 1) {
          processState = _FIRST;
        }
        else {
          processState = (ProcessState)((int)processState + 1);
        }
      }
      // Ask for the procces state to change to the previouse state or loop around to the end.
      else if(result.b == prevProcess) {
        if (processState == _FIRST) {
          processState = (ProcessState)((int)_LAST - 1);
        }
        else {
          processState = (ProcessState)((int)processState - 1);
        }
      }

      // Update the priorites of the controllers based upon the new process state.
      if (result.b == nextProcess || result.b == prevProcess) {
        ProcessData();
        result.b = wait;
        driveController.Reset(); // It is assumed that the drive controller may
                                 // be in a bad state if interrupted, so reset it.
      }
      break;
    }

    // Precision driving result types are when a controller wants direct
    // command of the robots actuators. LogicController facilitates the command
    // pass through in the LOGIC_STATE_PRECISION_COMMAND switch case.
    else if(result.type == precisionDriving) {

      logicState = LOGIC_STATE_PRECISION_COMMAND;
      break;

    }

    // Waypoints are also a pass through facilitated command but with a slightly
    // diffrent overhead. They are handled in the LOGIC_STATE_WAITING switch case.
    else if(result.type == waypoint) {

      logicState = LOGIC_STATE_WAITING;
      driveController.SetResultData(result);
      // Fall through on purpose to "case LOGIC_STATE_WAITING:"
    }

  }
  // ***************************************************************************
  // END LOGIC_STATE_INTERUPT
  // ***************************************************************************

  // ***************************************************************************
  // BEGIN LOGIC_STATE_WAITING
  // ***************************************************************************

  // This case is primarly when logic controller is waiting for drive controller
  // to reach its last waypoint.
  case LOGIC_STATE_WAITING: {
    // Ask drive controller how to drive: specifically, return commands to be
    // passed to the ROS Adapter such as left and right wheel PWM values in the
    // result struct.
    result = driveController.DoWork();

    // When out of waypoints, the drive controller will throw an interrupt.
    // However, unlike other controllers, drive controller is not on the
    // priority queue so it must be checked here.
    if (result.type == behavior) {
      if(driveController.ShouldInterrupt()) {
        logicState = LOGIC_STATE_INTERRUPT;
      }
    }
    break;
  }
  // ***************************************************************************
  // END LOGIC_STATE_WAITING
  // ***************************************************************************

  // ***************************************************************************
  // BEGIN LOGIC_STATE_PRECISION_COMMAND
  // ***************************************************************************

    // Used for precision driving pass through.
  case LOGIC_STATE_PRECISION_COMMAND: {

    // Unlike waypoints, precision commands change every update tick, so we ask
    // the controller for new commands on every update tick.
    result = control_queue.top().controller->DoWork();

    // Pass the driving commands to the drive controller so it can interpret them.
    driveController.SetResultData(result);

    // The interpreted commands are turned into proper initial_spiral_offset
    // motor commands to be passed the ROS Adapter such as left and right wheel
    // PWM values in the result struct.
    result = driveController.DoWork();
    break;

  }
  // ***************************************************************************
  // END LOGIC_STATE_PRECISION_COMMAND
  // ***************************************************************************
}
// end switch statment *********************************************************

  // Allow the controllers to communicate data between each other,
  // depending on the processState.
  controllerInterconnect();

  // Give the ROSAdapter the final decision on how it should drive.
  return result;
}

void LogicController::UpdateData()
{
  // As the top level controller, there is no specific data that must
  // be updated in this controller. This function may be of greater use
  // and importance in lower level controllers.
}

void LogicController::ProcessData()
{

  // This controller priority is used when searching.
  if (processState == PROCCESS_STATE_SEARCHING)
  {
    prioritizedControllers = {
      PrioritizedController{0, (Controller*)(&searchController)},
      PrioritizedController{10, (Controller*)(&obstacleController)},
      PrioritizedController{15, (Controller*)(&pickUpController)},
      PrioritizedController{5, (Controller*)(&range_controller)},
      PrioritizedController{-1, (Controller*)(&dropOffController)},
      PrioritizedController{-1, (Controller*)(&manualWaypointController)}
    };
  }

  // This priority is used when returning a target to the center collection zone.
  else if (processState  == PROCCESS_STATE_TARGET_PICKEDUP)
  {
    prioritizedControllers = {
    PrioritizedController{-1, (Controller*)(&searchController)},
    PrioritizedController{15, (Controller*)(&obstacleController)},
    PrioritizedController{-1, (Controller*)(&pickUpController)},
    PrioritizedController{10, (Controller*)(&range_controller)},
    PrioritizedController{1, (Controller*)(&dropOffController)},
    PrioritizedController{-1, (Controller*)(&manualWaypointController)}
    };
  }

  // This priority is used when returning a target to the center collection zone.
  else if (processState  == PROCCESS_STATE_DROP_OFF)
  {
    prioritizedControllers = {
      PrioritizedController{-1, (Controller*)(&searchController)},
      PrioritizedController{-1, (Controller*)(&obstacleController)},
      PrioritizedController{-1, (Controller*)(&pickUpController)},
      PrioritizedController{10, (Controller*)(&range_controller)},
      PrioritizedController{1, (Controller*)(&dropOffController)},
      PrioritizedController{-1, (Controller*)(&manualWaypointController)}
    };
  }

  // Under manual control ONLY the manual waypoint controller is active.
  else if (processState == PROCESS_STATE_MANUAL) {
    prioritizedControllers = {
      PrioritizedController{-1, (Controller*)(&searchController)},
      PrioritizedController{-1, (Controller*)(&obstacleController)},
      PrioritizedController{-1, (Controller*)(&pickUpController)},
      PrioritizedController{-1, (Controller*)(&range_controller)},
      PrioritizedController{-1, (Controller*)(&dropOffController)},
      PrioritizedController{5,  (Controller*)(&manualWaypointController)}
    };
  }
}

bool LogicController::ShouldInterrupt()
{
  ProcessData();

  // The logic controller is the top level controller and will never have to
  // interrupt. It is only the lower level controllers that may need to interupt.
  return false;
}

bool LogicController::HasWork()
{
  // The LogicController class is a special case. It will never have work to
  // do because it is always handling the work of the other controllers.
  return false;
}


// This function will deal with inter-controller communication. Communication
// that needs to occur between specific low level controllers is done here.
//
// The type of communication may or may not depend on the processState.
//
//                       /<----> ControllerA
// LogicController <---->|                  \__ inter-controller communication
//                       |                  /
//                       \<----> ControllerB
void LogicController::controllerInterconnect()
{

  if (processState == PROCCESS_STATE_SEARCHING)
  {

    // Obstacle controller needs to know if the center ultrasound should be ignored.
    if(pickUpController.GetIgnoreCenter())
    {
      obstacleController.setIgnoreCenterSonar();
    }

    // Pickup controller anounces it has picked up a target.
    if(pickUpController.GetTargetHeld())
    {
      dropOffController.SetTargetPickedUp();
      obstacleController.setTargetHeld();
      searchController.SetSuccesfullPickup();
    }
  }

  // Ask if drop off has released the target from the gripper yet.
  if (!dropOffController.HasTarget())
  {
    obstacleController.setTargetHeldClear();
  }

  // Obstacle controller is running and driveController needs to clear its waypoints.
  if(obstacleController.getShouldClearWaypoints())
  {
    driveController.Reset();
  }

}

// Receives position in the world inertial frame (should rename to SetOdomPositionData).
void LogicController::SetPositionData(Point currentLocation)
{
  searchController.SetCurrentLocation(currentLocation);
  dropOffController.SetCurrentLocation(currentLocation);
  obstacleController.setCurrentLocation(currentLocation);
  driveController.SetCurrentLocation(currentLocation);
  manualWaypointController.SetCurrentLocation(currentLocation);
}

// Recieves position in the world frame with global data (GPS).
void LogicController::SetMapPositionData(Point currentLocation)
{
  range_controller.setCurrentLocation(currentLocation);
}

// Sets the velocity data for the driveController. This information is
// necessary so that the drive controller can update the velocity sent
// back to the RosAdapter correctly.
void LogicController::SetVelocityData(float linearVelocity, float angularVelocity)
{
  driveController.SetVelocityData(linearVelocity,angularVelocity);
}

void LogicController::SetMapVelocityData(float linearVelocity, float angularVelocity)
{
}

// Give the specified controllers a list of visible april tags.
void LogicController::SetAprilTags(vector<Tag> tags)
{
  pickUpController.SetTagData(tags);
  obstacleController.setTagData(tags);
  dropOffController.SetTargetData(tags);
}

// Give the specified controllers the sonar sensor values.
void LogicController::SetSonarData(float left, float center, float right)
{
  // The pickUpController only needs the center data in order to tell if
  // an april tag cube has been picked up correctly.
  pickUpController.SetSonarData(center);

  obstacleController.setSonarData(left,center,right);
}

// Called once by RosAdapter in guarded init.
void LogicController::SetCenterLocationOdom(Point centerLocationOdom)
{
  searchController.SetCenterLocation(centerLocationOdom);
  dropOffController.SetCenterLocation(centerLocationOdom);
}

void LogicController::AddManualWaypoint(Point manualWaypoint, int waypoint_id)
{
  manualWaypointController.AddManualWaypoint(manualWaypoint, waypoint_id);
}

void LogicController::RemoveManualWaypoint(int waypoint_id)
{
  manualWaypointController.RemoveManualWaypoint(waypoint_id);
}

std::vector<int> LogicController::GetClearedWaypoints()
{
  return manualWaypointController.ReachedWaypoints();
}

void LogicController::setVirtualFenceOn( RangeShape* range )
{
  range_controller.setRangeShape(range);
  range_controller.setEnabled(true);
}

void LogicController::setVirtualFenceOff()
{
  range_controller.setEnabled(false);
}

void LogicController::SetCenterLocationMap(Point centerLocationMap)
{
}

void LogicController::SetCurrentTimeInMilliSecs( long int time )
{
  current_time = time;
  dropOffController.SetCurrentTimeInMilliSecs( time );
  pickUpController.SetCurrentTimeInMilliSecs( time );
  obstacleController.setCurrentTimeInMilliSecs( time );
}

void LogicController::SetModeAuto() {
  if(processState == PROCESS_STATE_MANUAL) {
    // only do something if we are in manual mode
    this->Reset();
    manualWaypointController.Reset();
  }
}
void LogicController::SetModeManual()
{
  if(processState != PROCESS_STATE_MANUAL) {
    logicState = LOGIC_STATE_INTERRUPT;
    processState = PROCESS_STATE_MANUAL;
    ProcessData();
    control_queue = priority_queue<PrioritizedController>();
    driveController.Reset();
  }
}
