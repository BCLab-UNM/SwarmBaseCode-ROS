#ifndef _POINT_HPP
#define _POINT_HPP

class Point
{
private:
   double _x;
   double _y;
   double _z;
public:
   /**
    * The default constructor makes a point that represents the origin
    * (0,0,0)
    */
   Point();

   /**
    * Create a point with the given x, y coordinates, on the xy
    * (ie. z=0).
    */
   Point(double x, double y);

   /**
    * Create a point with the given x, y, z coordinates
    */
   Point(double x, double y, double z);
   ~Point();

   void SetX(double x);
   void SetY(double y);
   void SetZ(double z);

   double GetX() const;
   double GetY() const;
   double GetZ() const;

   double Distance(const Point& p) const;
   bool WithinDistance(const Point& p, double d) const;

   bool  operator== (const Point&) const;
   Point operator+  (const Point&) const;
   Point operator-  (const Point&) const;
};

#endif // _POINT_HPP