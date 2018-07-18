#include "Point.hpp"

#include <cmath> // sqrt

Point::Point() :
   _x(0),
   _y(0),
   _z(0)
{}

Point::Point(double x, double y) :
   _x(x),
   _y(y),
   _z(0)
{}

Point::Point(double x, double y, double z) :
   _x(x),
   _y(y),
   _z(z)
{}

Point::~Point() {}

void Point::SetX(double x)
{
   _x = x;
}

void Point::SetY(double y)
{
   _y = y;
}

void Point::SetZ(double x)
{
   _z = x;
}

double Point::GetX() const
{
   return _x;
}

double Point::GetY() const
{
   return _y;
}

double Point::GetZ() const
{
   return _z;
}

double Point::Distance(const Point& p) const
{
   Point x = *this - p;
   return sqrt(x.GetX() * x.GetX() + x.GetY() * x.GetY() + x.GetZ() * x.GetZ());
}

bool Point::WithinDistance(const Point& p, double d) const
{
   return Distance(p) <= d;
}

bool Point::operator== (const Point& p) const
{
   return _x == p._x && _y == p._y && _z == p._z;
}

Point Point::operator- (const Point& p) const
{
   return Point(_x - p._x, _y - p._y, _z - p._z);
}

Point Point::operator+ (const Point& p) const
{
   return Point(_x + p._x, _y + p._y, _z + p._z);
}