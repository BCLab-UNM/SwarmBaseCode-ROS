#ifndef MAUNALWAYPOINTCONTROLLER_H
#define MANULAWAYPOINTCONTROLLER_H

#include <map>

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
   void AddManualWaypoint(Point wpt, int id);
   void RemoveManualWaypoint(int id);
   std::vector<int> ReachedWaypoints();
   
protected:
   void ProcessData() override;
   
private:
   Point currentLocation;
   // list of manual waypoints
   std::map<int,Point> waypoints;
   std::vector<int> cleared_waypoints;
   int num_waypoints = 0;

   // coppied from DriveController: 15 cm
   const float waypoint_tolerance = 0.15;
};

#endif // MANUALWAYPOINTCONTROLLER_H