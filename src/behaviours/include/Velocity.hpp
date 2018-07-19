#ifndef _VELOCITY_HPP
#define _VELOCITY_HPP

class Vector3
{
private:
   double _x;
   double _y;
   double _z;
public:
   Vector3(double x, double y, double z) :
      _x(x), _y(y), _z(z)
      {}

   ~Vector3() {}

   double GetX() const { return _x; }
   double GetY() const { return _y; }
   double GetZ() const { return _z; }

   void SetX(double x) { _x = x; }
   void SetY(double y) { _y = y; }
   void SetZ(double z) { _z = z; }

   double Magnitude() const;

   friend bool operator== (const Vector3& v, const Vector3& u);
   friend Vector3 operator* (double x, const Vector3& v);
   // TODO: dot-product.
   Vector3 operator+ (const Vector3& v);
   Vector3 operator- (const Vector3& v);
};

/**
 * Represents a linear velocity as the mognitude of its x, y, and z
 * components.
 */
class LinearVelocity
{
private:
   Vector3 _velocity;
public:
   /**
    * The default constructor creates a velocity of 0 m/s in
    */
   LinearVelocity();

   /**
    * Create a linear velocity using a Vector3
    */
   LinearVelocity(Vector3 v);

   /**
    * Create a velocity only in the x direction.
    */
   LinearVelocity(double x);

   /**
    * Create a velocity with only x and y components.
    */
   LinearVelocity(double x, double y);

   /**
    * Create a velocity with x, y, and z components.
    */
   LinearVelocity(double x, double y, double z);

   ~LinearVelocity();

   /**
    * Set the magnitude of the x component of the velocity.
    */
   void SetX(double x);

   /**
    * Set the magnitude of the y component of the velocity.
    */
   void SetY(double y);

   /**
    * Set the magnitude of the z component of the velocity.
    */
   void SetZ(double x);

   /**
    * Get the components of the velocity.
    */
   double GetX() const;
   double GetY() const;
   double GetZ() const;

   /**
    * Get the magnitude of the velocity vector.
    */
   double GetMagnitude() const;

   LinearVelocity operator+  (const LinearVelocity& vel);
   LinearVelocity operator-  (const LinearVelocity& vel);
   friend bool operator== (const LinearVelocity& v1, const LinearVelocity& v2);

   /**
    * Scale the velocity vector.
    */
   friend LinearVelocity operator* (const double x, const LinearVelocity& vel);
};

/**
 * An angular velocity represented by roll pitch and yaw components.
 */
class AngularVelocity
{
private:
   Vector3 _velocity;
public:
   /**
    * Create an angular volocity with magnitued 0.
    */
   AngularVelocity();

   /**
    * Create an angular velocity from a vector.
    */
   AngularVelocity(Vector3 v);

   /**
    * Create an angular velocity with the given magnitude in each of
    * roll, pitch, and yaw in radians per second.
    */
   AngularVelocity(double roll, double pitch, double yaw);
   ~AngularVelocity();

   void SetRoll(double roll);
   void SetPitch(double pitch);
   void SetYaw(double yaw);

   double GetRoll() const;
   double GetPitch() const;
   double GetYaw() const;
   double GetMagnitude() const;

   AngularVelocity operator+  (const AngularVelocity& vel);
   AngularVelocity operator-  (const AngularVelocity& vel);
   friend bool operator== (const AngularVelocity& v1, const AngularVelocity& v2);
   friend AngularVelocity operator* (const double x, const AngularVelocity& vel);
};

#endif // _VELOCITY_HPP