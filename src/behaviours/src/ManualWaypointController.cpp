#include "ManualWaypointController.h"
#include <angles/angles.h> // for hypot()

ManualWaypointController::ManualWaypointController() {
   
}

ManualWaypointController::~ManualWaypointController() {}

void ManualWaypointController::Reset() {
   waypoints.waypoints.clear();
}

bool ManualWaypointController::HasWork() {
   return !waypoints.waypoints.empty();
}

bool ManualWaypointController::ShouldInterrupt() {
   // If the size of the manual waypoint list has changed, then interrupt.
   return interrupt;
}

Result ManualWaypointController::DoWork() {
   Result result;
   result.type = waypoint;
   result.wpts.waypoints.push_back(waypoints.waypoints.front());

   return result;
}

void ManualWaypointController::SetCurrentLocation(Point currentLocation)
{
   this->currentLocation = currentLocation;
   if(!waypoints.waypoints.empty()) {
      if(hypot(waypoints.waypoints.front().x-currentLocation.x,
               waypoints.waypoints.front().y-currentLocation.y)
         < waypoint_tolerance) {
         waypoints.waypoints.erase(waypoints.waypoints.begin());
         // interrupt any time the waypoint list changes and is non-empty.
         if(!waypoints.waypoints.empty()) {
            interrupt = true;
         }
      }
   }
}

void ManualWaypointController::ProcessData()
{   
}

void ManualWaypointController::AddManualWaypoint(Point wpt)
{
   waypoints.waypoints.push_back(wpt);
   // interrupt any time the waypoint list changes and is non-empty.
   interrupt = true;
}
