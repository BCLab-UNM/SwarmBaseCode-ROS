#ifndef TAG_H
#define TAG_H

#include <boost/math/quaternion.hpp> // For quaternion
#include <tuple> // For packaging position and orientation values

// Stores AprilTag data
class Tag {
 public:

  // Contructors
  Tag();
  Tag( const Tag &that);
  
  // Getters and setters
  int getID();
  void setID( int ); 

  // Get and set positions as <x,y,z> tuples
  std::tuple<float, float, float> getPosition();
  void setPosition( std::tuple<float, float, float> );
  
  // Get and set orientation as a <x,y,z,w> quaternion
  ::boost::math::quaternion<float> getOrientation();
  void setOrientation( ::boost::math::quaternion<float> );

  // convenience accessor functions
  float getPositionX();
  float getPositionY();
  float getPositionZ();
  float getOrientationX();
  float getOrientationY();
  float getOrientationZ();
  float getOrientationW();

  void setPositionX( float );
  void setPositionY( float );
  void setPositionZ( float );
  void setOrientationX( float );
  void setOrientationY( float );
  void setOrientationZ( float );
  void setOrientationW( float );
  
  
 private:

  // Tag ID
  int id = 0;
  
  // Position in 3D coords
  std::tuple<float,float,float> position;

  // Orientation as a quaternion
  ::boost::math::quaternion<float> orientation;
};

#endif // TAG_H
