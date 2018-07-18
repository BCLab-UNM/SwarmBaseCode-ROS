#ifndef _ACTION_HPP
#define _ACTION_HPP

#include "Velocity.hpp"
#include "Point.hpp"

/**
 * An action defined by a linear and an angular velocity
 */
class VelocityAction
{
private:
   LinearVelocity  _linear;
   AngularVelocity _angular;
public:
   VelocityAction();
   VelocityAction(LinearVelocity linear);
   VelocityAction(AngularVelocity angular);
   VelocityAction(LinearVelocity linear, AngularVelocity angular);
   ~VelocityAction();

   double SetRoll(double yaw);
   double SetPitch(double pitch);
   double SetYaw(double yaw);

   double SetX(double x);
   double SetY(double y);
   double SetZ(double z);

   void SetLinear(LinearVelocity linear);
   void SetAngular(AngularVelocity angular);

   double GetAngularMagnitude() const;
   double GetLinearMagnitude() const;

};

/**
 * An action that defines a location in space the robot should move
 * towards.
 */
class WaypointAction
{
private:
   Point  _waypoint;
   double _speed = 0.75; // meters per second
   double _tolerance = 0.15;    // 0.15 meters
public:
   WaypointAction(Point w);
   ~WaypointAction();

   Point  GetWaypoint()  const;
   double GetSpeed()     const;
   double GetTolerance() const;

   void SetTolerance(double t);
   void SetSpeed(double s);
   void SetWaypoint(Point w);
   void SetX(double x);
   void SetY(double y);
   void SetZ(double z);

   bool IsAt(const Point&) const;
};

#endif // _ACTION_HPP