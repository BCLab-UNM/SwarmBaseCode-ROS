#ifndef MAUNALWAYPOINTCONTROLLER_H
#define MANULAWAYPOINTCONTROLLER_H

#include "Controller.h"

class ManualWaypointController : virtual Controller
{
public:
   ManualWaypointController();
   ~ManualWaypointController();

   void Reset() override;
   Result DoWork() override;
   bool HasWork() override;
   bool ShouldInterrupt() override;

   void SetCurrentLocation(Point currentLocation);
   void AddManualWaypoint(Point wpt);
   
protected:
   void ProcessData() override;
   
private:
   Point currentLocation;
   // list of manual waypoints
   Waypoints waypoints;
   int num_waypoints = 0;

   // coppied from DriveController: 15 cm
   const float waypoint_tolerance = 0.15;
};

#endif // MANUALWAYPOINTCONTROLLER_H