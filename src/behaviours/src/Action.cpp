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

   void VelocityAction::SetRoll(double roll)
   {
      _angular.SetRoll(roll);
   }

   void VelocityAction::SetPitch(double pitch)
   {
      _angular.SetPitch(pitch);
   }

   void VelocityAction::SetYaw(double yaw)
   {
      _angular.SetYaw(yaw);
   }

   void VelocityAction::SetX(double x)
   {
      _linear.SetX(x);
   }

   void VelocityAction::SetY(double y)
   {
      _linear.SetY(y);
   }

   void VelocityAction::SetZ(double z)
   {
      _linear.SetZ(z);
   }

   double VelocityAction::GetAngularMagnitude() const
   {
      return _angular.GetMagnitude();
   }

   double VelocityAction::GetLinearMagnitude() const
   {
      return _linear.GetMagnitude();
   }

   double VelocityAction::GetX() const
   {
      return _linear.GetX();
   }

   double VelocityAction::GetY() const
   {
      return _linear.GetY();
   }

   double VelocityAction::GetZ() const
   {
      return _linear.GetX();
   }

   double VelocityAction::GetRoll() const
   {
      return _angular.GetRoll();
   }

   double VelocityAction::GetPitch() const
   {
      return _angular.GetPitch();
   }

   double VelocityAction::GetYaw() const
   {
      return _angular.GetYaw();
   }

   void VelocityAction::SetAngular(AngularVelocity v)
   {
      _angular = v;
   }

   void VelocityAction::SetLinear(LinearVelocity v)
   {
      _linear = v;
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
      _type(Type::VELOCITY),
      _velocity(v)
   {}

   Action::Action(WaypointAction w) :
      _type(Action::WAYPOINT),
      _waypoint(w)
   {}

   Action::Action() :
      _type(Type::VELOCITY),
      _velocity(VelocityAction(LinearVelocity(0)))
   {}

   VelocityAction Action::GetVelocity() const throw(BadType)
   {
      if(_type != Type::VELOCITY)
      {
         throw BadType();
      }
      return _velocity;
   }

   WaypointAction Action::GetWaypoint() const throw(BadType)
   {
      if(_type != Type::WAYPOINT)
      {
         throw BadType();
      }
      return _waypoint;
   }

   Action::Type Action::GetType() const
   {
      return _type;
   }

   void Action::SetAction(WaypointAction w)
   {
      _type = Type::WAYPOINT;
      _waypoint = w;
   }

   void Action::SetAction(VelocityAction v)
   {
      _type = Type::VELOCITY;
      _velocity = v;
   }
}