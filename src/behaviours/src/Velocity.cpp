#include "Velocity.hpp"

#include <cmath> // sqrt()

bool operator== (const Vector3& v, const Vector3& u)
{
   return v._x == u._x && v._y == u._y && v._z == u._z;
}

Vector3 operator* (double x, const Vector3& v)
{
   return Vector3(x*v._x, x*v._y, x*v._z);
}

Vector3 Vector3::operator+(const Vector3& v)
{
   return Vector3(_x + v._x, _y + v._y, _z + v._z);
}

Vector3 Vector3::operator-(const Vector3& v)
{
   return Vector3(_x - v._x, _y - v._y, _z - v._z);
}

double Vector3::Magnitude() const
{
   return sqrt(_x*_x + _y*_y + _z*_z);
}

/// Linear velocity.

LinearVelocity::LinearVelocity() :
   _velocity(0, 0, 0)
{}

LinearVelocity::LinearVelocity(double x) :
   _velocity(x, 0, 0)
{}

LinearVelocity::LinearVelocity(double x, double y) :
   _velocity(x, y, 0)
{}

LinearVelocity::LinearVelocity(double x, double y, double z) :
   _velocity(x, y, z)
{}

LinearVelocity::LinearVelocity(Vector3 v) :
   _velocity(v)
{}

LinearVelocity::~LinearVelocity() {}

void LinearVelocity::SetX(double x)
{
   _velocity.SetX(x);
}

void LinearVelocity::SetY(double y)
{
   _velocity.SetY(y);
}

void LinearVelocity::SetZ(double z)
{
   _velocity.SetZ(z);
}

double LinearVelocity::GetX() const
{
   return _velocity.GetX();
}

double LinearVelocity::GetY() const
{
   return _velocity.GetY();
}

double LinearVelocity::GetZ() const
{
   return _velocity.GetZ();
}

double LinearVelocity::GetMagnitude() const
{
   return _velocity.Magnitude();
}

LinearVelocity LinearVelocity::operator+(const LinearVelocity& vel)
{
   return LinearVelocity(_velocity + vel._velocity);
}

LinearVelocity LinearVelocity::operator-(const LinearVelocity& vel)
{
   return LinearVelocity(_velocity - vel._velocity);
}

bool operator==(const LinearVelocity& v1, const LinearVelocity& v2)
{
   return v1._velocity == v2._velocity;
}

LinearVelocity operator*(const double x, const LinearVelocity& vel)
{
   return LinearVelocity(x*vel._velocity);
}

/// AngularVelocity

AngularVelocity::AngularVelocity() :
   _velocity(0,0,0)
{}

AngularVelocity::AngularVelocity(Vector3 v) :
   _velocity(v)
{}

AngularVelocity::AngularVelocity(double roll, double pitch, double yaw) :
   _velocity(roll, pitch, yaw)
{}

AngularVelocity::~AngularVelocity() {}

void AngularVelocity::SetRoll(double roll)
{
   _velocity.SetX(roll);
}

void AngularVelocity::SetPitch(double pitch)
{
   _velocity.SetY(pitch);
}

void AngularVelocity::SetYaw(double yaw)
{
   _velocity.SetZ(yaw);
}

double AngularVelocity::GetRoll() const
{
   return _velocity.GetX();
}

double AngularVelocity::GetPitch() const
{
   return _velocity.GetY();
}

double AngularVelocity::GetYaw() const
{
   return _velocity.GetZ();
}

double AngularVelocity::GetMagnitude() const
{
   return _velocity.Magnitude();
}

AngularVelocity AngularVelocity::operator+(const AngularVelocity& vel)
{
   return AngularVelocity(_velocity + vel._velocity);
}

AngularVelocity AngularVelocity::operator-(const AngularVelocity& vel)
{
   return AngularVelocity(_velocity - vel._velocity);
}

bool operator==(const AngularVelocity &v1, const AngularVelocity& v2)
{
   return v1._velocity == v2._velocity;
}

AngularVelocity operator* (const double x, const AngularVelocity& vel)
{
   return AngularVelocity(x*vel._velocity);
}
