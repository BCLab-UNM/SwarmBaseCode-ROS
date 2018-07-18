#include "Velocity.hpp"

#include <cmath> // sqrt()

LinearVelocity::LinearVelocity() :
   _x(0.0),
   _y(0.0),
   _z(0.0)
{}

LinearVelocity::LinearVelocity(double x) :
   _x(x),
   _y(0.0),
   _z(0.0)
{}

LinearVelocity::LinearVelocity(double x, double y) :
   _x(x),
   _y(y),
   _z(0.0)
{}

LinearVelocity::LinearVelocity(double x, double y, double z) :
   _x(x),
   _y(y),
   _z(z)
{}

LinearVelocity::~LinearVelocity() {}

void LinearVelocity::SetX(double x)
{
   _x = x;
}

void LinearVelocity::SetY(double y)
{
   _y = y;
}

void LinearVelocity::SetZ(double z)
{
   _z = z;
}

double LinearVelocity::GetX() const
{
   return _x;
}

double LinearVelocity::GetY() const
{
   return _y;
}

double LinearVelocity::GetZ() const
{
   return _z;
}

double LinearVelocity::GetMagnitude() const
{
   return sqrt(_x*_x + _y*_y + _z*_z);
}

LinearVelocity LinearVelocity::operator+(const LinearVelocity& vel)
{
   return LinearVelocity(vel.GetX() + _x,
                         vel.GetY() + _y,
                         vel.GetZ() + _z);
}

LinearVelocity LinearVelocity::operator-(const LinearVelocity& vel)
{
   return LinearVelocity(_x - vel.GetX(),
                         _y - vel.GetY(),
                         _z - vel.GetZ());
}

bool operator==(const LinearVelocity& v1, const LinearVelocity& v2)
{
   return (v1._x == v2._x)
      && (v1._y  == v2._y)
      && (v1._z  == v2._z);
}

LinearVelocity operator*(const double x, const LinearVelocity& vel)
{
   return LinearVelocity(x*vel._x, x*vel._y, x*vel._z);
}

AngularVelocity::AngularVelocity() :
   _roll(0),
   _pitch(0),
   _yaw(0)
{}

AngularVelocity::AngularVelocity(double roll, double pitch, double yaw) :
   _roll(roll),
   _pitch(pitch),
   _yaw(yaw)
{}

AngularVelocity::~AngularVelocity() {}

void AngularVelocity::SetRoll(double roll)
{
   _roll = roll;
}

void AngularVelocity::SetPitch(double pitch)
{
   _pitch = pitch;
}

void AngularVelocity::SetYaw(double yaw)
{
   _yaw = yaw;
}

double AngularVelocity::GetRoll() const
{
   return _roll;
}

double AngularVelocity::GetPitch() const
{
   return _pitch;
}

double AngularVelocity::GetYaw() const
{
   return _yaw;
}

double AngularVelocity::GetMagnitude() const
{
   return sqrt(_roll*_roll + _pitch*_pitch + _yaw*_yaw);
}

AngularVelocity AngularVelocity::operator+(const AngularVelocity& vel)
{
   return AngularVelocity(_roll + vel.GetRoll(),
                          _pitch + vel.GetPitch(),
                          _yaw + vel.GetYaw());
}

AngularVelocity AngularVelocity::operator-(const AngularVelocity& vel)
{
   return AngularVelocity(_roll - vel.GetRoll(),
                          _pitch - vel.GetPitch(),
                          _yaw - vel.GetYaw());
}

bool operator==(const AngularVelocity &v1, const AngularVelocity& v2)
{
   return (v1._roll == v2._roll)
      && (v1._pitch == v2._pitch)
      && (v1._yaw == v1._yaw);
}

AngularVelocity operator* (const double x, const AngularVelocity& vel)
{
   return AngularVelocity(x*vel._roll, x*vel._pitch, x*vel._yaw);
}
