#ifndef LOGICCONTROLLER_H
#define LOGICCONTROLLER_H

#include "Controller.h"
#include "PickUpController.h"
#include "DropOffController.h"
#include "SearchController.h"
#include "ObstacleController.h"
#include "DriveController.h"
#include "RangeController.h"
#include "ManualWaypointController.h"

#include <vector>
#include <queue>

using namespace std;

struct PrioritizedController {
  int priority = -1;
  Controller* controller = nullptr;

  PrioritizedController(int pri, Controller* cntrl) : priority(pri), controller(cntrl) {}

  inline bool operator <(const PrioritizedController& other) const {
    return priority < other.priority;
  }
};

class LogicController : virtual Controller
{
public:
  LogicController();
  ~LogicController();

  void Reset() override;
  Result DoWork() override;
  void UpdateData();
  bool ShouldInterrupt() override;
  bool HasWork() override;

  void SetAprilTags(vector<Tag> tags);
  void SetSonarData(float left, float center, float right);
  void SetPositionData(Point currentLocation);
  void SetMapPositionData(Point currentLocationMap);
  void SetVelocityData(float linearVelocity, float angularVelocity);
  void SetMapVelocityData(float linearVelocity, float angularVelocity);
  void SetCenterLocationOdom(Point centerLocationOdom);
  void SetCenterLocationMap(Point centerLocationMap);

  
  // Passthrough for providing new waypoints to the
  // ManualWaypointController.
  void AddManualWaypoint(Point wpt, int waypoint_id);

  
  // Passthrough for removing waypoints from the
  // ManualWaypointController.
  void RemoveManualWaypoint(int waypoint_id);

  
  // Passthrough for getting the list of manual waypoints that have
  // been visited. 
  std::vector<int> GetClearedWaypoints();

  
  // Put the logic controller into manual mode. Changes process state
  // to PROCESS_STATE_MANUAL and logic state to LOGIC_STATE_INTERRUPT.
  
  // If the logic controller is already in manual mode this has no
  // effect.
  void SetModeManual();

  
  // Put the logic controller into autonomous mode. Resets the logic
  // controller and clears all manual waypoints.
  //
  // If the logic controller is already in autonomous mode, then this
  // has no effect.
  void SetModeAuto();

  void SetCurrentTimeInMilliSecs( long int time );

  // Tell the logic controller whether rovers should automatically
  // resstrict their foraging range. If so provide the shape of the
  // allowed range.
  void setVirtualFenceOn( RangeShape* range );
  void setVirtualFenceOff( );

protected:
  void ProcessData();

private:

  enum LogicState {
    LOGIC_STATE_INTERRUPT = 0,
    LOGIC_STATE_WAITING,
    LOGIC_STATE_PRECISION_COMMAND
  };

  enum ProcessState {
    _FIRST = 0,
    PROCCESS_STATE_SEARCHING = 0,
    PROCCESS_STATE_TARGET_PICKEDUP,
    PROCCESS_STATE_DROP_OFF,
    _LAST,
    PROCESS_STATE_MANUAL // robot is under manual control
  };

  LogicState logicState;
  ProcessState processState;

  PickUpController pickUpController;
  DropOffController dropOffController;
  SearchController searchController;
  ObstacleController obstacleController;
  DriveController driveController;
  RangeController range_controller;
  ManualWaypointController manualWaypointController;

  std::vector<PrioritizedController> prioritizedControllers;
  priority_queue<PrioritizedController> control_queue;

  void controllerInterconnect();

  long int current_time = 0;
};

#endif // LOGICCONTROLLER_H
