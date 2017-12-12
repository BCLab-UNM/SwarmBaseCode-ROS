#ifndef MAUNALWAYPOINTCONTROLLER_H
#define MANULAWAYPOINTCONTROLLER_H

#include <map>

#include "Controller.h"

class ManualWaypointController : virtual Controller
{
public:
   ManualWaypointController();
   ~ManualWaypointController();

   
   // Clears the list of waypoints to visit.
    
   // NOTE: this will not stop the robot from driving to a waypoint if
   // it was already driving there before this was called. 
   void Reset() override;

   
   // Returns the next waypoint in the list. The result is set up to
   // be used by DriveController for waypoint navigation.
   //
   Result DoWork() override;

   
   // True if there are waypoints in the list. False otherwise.
   bool HasWork() override;
   
   // Interrupts only if the number of waypoints has changed and is
   // non-zero.
   bool ShouldInterrupt() override;

   
   // Tell the controller the current location of the robot.
   void SetCurrentLocation(Point currentLocation);

   // Add the provided waypoint to the list of manual waypoints.
   
   // NOTE: Waypoints should have unique ids, it is incumbent on the
   // caller to ensure this. Providing the same IDs for multiple
   // waypoints may cause undefined behavior in the GUI. Further more
   // if the ID is removed using RemoveManualWaypoint() then all
   // waypoints with that ID may be removed.
   void AddManualWaypoint(Point wpt, int id);

   
   // Remove the waypoint with the given ID from the list of waypoints
   // to visit. If no maypoint exists with the given ID, the no action
   // is taken.
   void RemoveManualWaypoint(int id);

   // Get a vector containing all waypoint IDs that have been visited
   // since this function was last called.
   
   // This should be called regularly to prevent memory leaks.
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
