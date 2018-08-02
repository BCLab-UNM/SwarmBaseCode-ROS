#include "Heading.hpp"

#include <cmath> // PI related constants

Heading::Heading() : _heading(0) {}

Heading::Heading(double h)
{
   // put the heading in to the range [0,2PI]
   _heading = h - floor(h/(2*M_PI)) * (2*M_PI);
}

Heading::~Heading() {}

double Heading::GetHeadingRadians() const
{
   return _heading;
}

Heading Heading::operator+(const Heading& h)
{
   return Heading(_heading + h._heading);
}

Heading Heading::operator-(const Heading& h)
{
   return Heading(_heading - h._heading);
}

bool operator==(const Heading& g, const Heading& h)
{
   return h._heading == g._heading;
}

