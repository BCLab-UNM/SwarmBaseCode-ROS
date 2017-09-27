#include "Tag.h"

using namespace std;
using namespace boost::math;

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

int Tag::getID() {
  return id;
}

void Tag::setID( int id) {
  this->id = id;
}

tuple<float, float, float> Tag::getPosition() {
  return position;
}

void Tag::setPosition( tuple<float, float, float> position ) {
  this->position = position;
}

quaternion<float> Tag::getOrientation() {
  return orientation;
}

void Tag::setOrientation( quaternion<float> orientation ) {
  this->orientation = orientation;
}

// convenience accessor functions
float Tag::getPositionX(){
  return std::get<0>(position);
}

float Tag::getPositionY(){
  return std::get<1>(position);
}

float Tag::getPositionZ(){
  return std::get<2>(position);
}

float Tag::getOrientationX(){
  return orientation.R_component_1();
}

float Tag::getOrientationY(){
  orientation.R_component_2();
}

float Tag::getOrientationZ(){
  orientation.R_component_3();
}

float Tag::getOrientationW(){
  orientation.R_component_4();
}

void Tag::setPositionX( float x ){
  std::get<0>(position) = x;
}

void Tag::setPositionY( float y ){
  std::get<1>(position) = y;
}

void Tag::setPositionZ( float z ){
  std::get<2>(position) = z;
}

// The following function recreate the entire quaternion each time a value is changed
// becuase you cannot change quaternion values individually
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
