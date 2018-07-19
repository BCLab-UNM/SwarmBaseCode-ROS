#include "Action.hpp"

namespace core {

   VelocityAction::VelocityAction()
   {}

   VelocityAction::VelocityAction(LinearVelocity linear) :
      _linear(linear)
   {}

   VelocityAction::VelocityAction(AngularVelocity angular) :
      _angular(angular)
   {}

   VelocityAction::VelocityAction(LinearVelocity linear, AngularVelocity angular) :
      _angular(angular),
      _linear(linear)
   {}

   VelocityAction::~VelocityAction() {}

   double VelocityAction::SetRoll(double roll)
   {
      _angular.SetRoll(roll);
   }

   double VelocityAction::SetPitch(double pitch)
   {
      _angular.SetPitch(pitch);
   }

   double VelocityAction::SetYaw(double yaw)
   {
      _angular.SetYaw(yaw);
   }

   double VelocityAction::SetX(double x)
   {
      _linear.SetX(x);
   }

   double VelocityAction::SetY(double y)
   {
      _linear.SetY(y);
   }

   double VelocityAction::SetZ(double z)
   {
      _linear.SetZ(z);
   }

   double VelocityAction::GetAngularMagnitude() const
   {
      _angular.GetMagnitude();
   }

   double VelocityAction::GetLinearMagnitude() const
   {
      _linear.GetMagnitude();
   }

   WaypointAction::WaypointAction(Point w) :
      _waypoint(w)
   {}

   WaypointAction::~WaypointAction() {}

   Point WaypointAction::GetWaypoint() const
   {
      return _waypoint;
   }

   double WaypointAction::GetSpeed() const
   {
      return _speed;
   }

   double WaypointAction::GetTolerance() const
   {
      return _tolerance;
   }

   double WaypointAction::GetX() const
   {
      return _waypoint.GetX();
   }

   double WaypointAction::GetY() const
   {
      return _waypoint.GetY();
   }

   double WaypointAction::GetZ() const
   {
      return _waypoint.GetZ();
   }

   void WaypointAction::SetTolerance(double t)
   {
      _tolerance = t;
   }

   void WaypointAction::SetSpeed(double s)
   {
      _speed = s;
   }

   void WaypointAction::SetWaypoint(Point w)
   {
      _waypoint = w;
   }

   void WaypointAction::SetX(double x)
   {
      _waypoint.SetX(x);
   }

   void WaypointAction::SetY(double y)
   {
      _waypoint.SetY(y);
   }

   void WaypointAction::SetZ(double z)
   {
      _waypoint.SetZ(z);
   }

   bool WaypointAction::IsAt(const Point& p) const
   {
      return _waypoint.WithinDistance(p, _tolerance);
   }

   /// Composite action implementation

   Action::Action(VelocityAction v) :
      _type(ActionType::VELOCITY),
      _velocity(v)
   {}

   Action::Action(WaypointAction w) :
      _type(Action::WAYPOINT),
      _waypoint(w)
   {}

   VelocityAction Action::GetVelocity() const throw(BadType)
   {
      if(_type != ActionType::VELOCITY)
      {
         throw BadType();
      }
      return _velocity;
   }

   WaypointAction Action::GetWaypoint() const throw(BadType)
   {
      if(_type != ActionType::WAYPOINT)
      {
         throw BadType();
      }
      return _waypoint;
   }

   void Action::SetAction(WaypointAction w)
   {
      _type = ActionType::WAYPOINT;
      _waypoint = w;
   }

   void Action::SetAction(VelocityAction v)
   {
      _type = ActionType::VELOCITY;
      _velocity = v;
   }
}