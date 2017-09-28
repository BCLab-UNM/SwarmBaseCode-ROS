#ifndef TAG_H
#define TAG_H

#include <boost/math/quaternion.hpp> // For quaternion
#include <tuple> // For packaging position and orientation values
#include <iostream> // For output streams

// Stores AprilTag data
class Tag {
 public:

  // Contructors
  Tag();
  Tag( const Tag &that);
  
  // Getters and setters
  int getID() const;
  void setID( int ); 

  // Get and set positions as <x,y,z> tuples
  std::tuple<float, float, float> getPosition() const;;
  void setPosition( std::tuple<float, float, float> );
  
  // Get and set orientation as a <x,y,z,w> quaternion
  ::boost::math::quaternion<float> getOrientation() const;;
  void setOrientation( ::boost::math::quaternion<float> );

  // convenience accessor functions. The const keyword promises that we will not modify
  // class data as a side-effect of getting data.
  float getPositionX() const;
  float getPositionY() const;
  float getPositionZ() const;
  float getOrientationX() const;
  float getOrientationY() const;
  float getOrientationZ() const;
  float getOrientationW() const;

  void setPositionX( float );
  void setPositionY( float );
  void setPositionZ( float );
  void setOrientationX( float );
  void setOrientationY( float );
  void setOrientationZ( float );
  void setOrientationW( float );
  
  std::tuple<float,float,float> calcRollPitchYaw() const;;
  float calcRoll() const;
  float calcPitch() const;
  float calcYaw() const;
  
  // Allow writing of tag data to an output stream by  extending ostream.
  friend std::ostream& operator<<(std::ostream&, const Tag& );
  

  
  private:

  // Tag ID
  int id = 0;
  
  // Position in 3D coords
  std::tuple<float,float,float> position;

  // Orientation as a quaternion
  ::boost::math::quaternion<float> orientation;
};

#endif // TAG_H
