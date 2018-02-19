#include "Tag.h"

#include <cmath> // For trig functions

using namespace std;
using namespace boost::math;

// Extend ostream.
// Output stream operator. Allows easy writing of tag data to an output stream.
ostream& operator<<(ostream& output_stream, const Tag& tag) {

  tuple<float, float, float> position = tag.getPosition();
  tuple<float, float, float> orientation = tag.calcRollPitchYaw();
  
  output_stream << "Tag ID:" << tag.getID()
		<< ", Position (XYZ): <"
		<< get<0>(position) << ", "
		<< get<1>(position) << ", "
		<< get<2>(position) << ", "
		
		<< ">, "
		<< " Orientation (RPY): <"
		<< get<0>(orientation) << ", "
		<< get<1>(orientation) << ", "
		<< get<2>(orientation) << ", "
		<< ">"
		<< endl;
  
  return output_stream;
}


// Constructors
Tag::Tag() {
  // Initialize orientation and position
  orientation = ::boost::math::quaternion<float>(0,0,0,0);
  position = make_tuple(0,0,0);
}

Tag::Tag(const Tag &that) {
  this->setPosition( that.position );
  this->setOrientation( that.orientation );
  this->setID( that.id );
}

int Tag::getID() const {	//returns april tag ID
  return id;
}

void Tag::setID( int id) {	//sets april tag ID
  this->id = id;
}

tuple<float, float, float> Tag::getPosition() const {	//returns a position with X, Y, and Z coordinates
  return position;
}

void Tag::setPosition( tuple<float, float, float> position ) {	//sets X, Y, Z coordinates of tag
  this->position = position;
}

quaternion<float> Tag::getOrientation() const {	//returns tags orientation
  return orientation;
}

void Tag::setOrientation( quaternion<float> orientation ) {	//sets tags orientation
  this->orientation = orientation;
}

// convenience accessor functions
float Tag::getPositionX() const {
  return std::get<0>(position);
}

float Tag::getPositionY() const {
  return std::get<1>(position);
}

float Tag::getPositionZ() const {
  return std::get<2>(position);
}

float Tag::getOrientationX() const {
  return orientation.R_component_1();
}

float Tag::getOrientationY() const {
  orientation.R_component_2();
}

float Tag::getOrientationZ() const {
  orientation.R_component_3();
}

float Tag::getOrientationW() const {
  orientation.R_component_4();
}

void Tag::setPositionX( float x ) {
  std::get<0>(position) = x;
}

void Tag::setPositionY( float y ) {
  std::get<1>(position) = y;
}

void Tag::setPositionZ( float z ) {
  std::get<2>(position) = z;
}

// The following functions recreate the entire quaternion each time a value is changed
// becuase you cannot change quaternion components without rebuilding the whole quaternion.
// This is a limitation of the boost quaternion.
void Tag::setOrientationX( float x ){
  orientation = quaternion<float>( x, getOrientationY(), getOrientationZ(), getOrientationW() );
}

void Tag::setOrientationY( float y ){
  orientation = quaternion<float>( getOrientationX(), y, getOrientationZ(), getOrientationW() );
}

void Tag::setOrientationZ( float z ){
  orientation = quaternion<float>( getOrientationX(), getOrientationY(), z, getOrientationW() );
}

void Tag::setOrientationW( float w ){
  orientation = quaternion<float>( getOrientationX(), getOrientationY(), getOrientationZ(), w );
}

// ***
// Consider whether to memoise the "calc" functions for performace
// ***

// Returns orientation as Roll-Pitch-Yaw
tuple<float, float, float> Tag::calcRollPitchYaw() const {

  float yaw = calcYaw();
  float pitch = calcPitch();
  float roll = calcRoll();

  return make_tuple(roll, pitch, yaw);
}

float Tag::calcYaw() const {	//returns a yaw value helpful for determining the angle at which the tag is being viewed

  float x = getOrientationX();
  float y = getOrientationY();
  float z = getOrientationZ();
  float w = getOrientationW();

  return atan2(2.0f*(y*z + w*x), w*w - x*x - y*y + z*z);
}

float Tag::calcPitch() const {

  float x = getOrientationX();
  float y = getOrientationY();
  float z = getOrientationZ();
  float w = getOrientationW();

  return asin(-2.0f*(x*z - w*y));
}

float Tag::calcRoll() const { 

  float x = getOrientationX();
  float y = getOrientationY();
  float z = getOrientationZ();
  float w = getOrientationW();

  return atan2(2.0f*(y + w*z), w*w + x*x - y*y - z*z);
}


