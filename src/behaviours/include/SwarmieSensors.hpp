#ifndef _SWARMIE_SENSORS_HPP
#define _SWARMIE_SENSORS_HPP

#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <boost/math/quaternion.hpp>
#include <cmath> // atan2

class Tag
{
private:
   int _id;
   double _x;
   double _y;
   double _z;
   boost::math::quaternion<double> _orientation;

   const double CAMERA_HEIGHT = 0.195;
   const double CAMERA_OFFSET = 0.023;
public:
   Tag(int id, double x, double y, double z, boost::math::quaternion<double> orientation);
   ~Tag() {}
   
   double Alignment() const;
   double Distance() const;
   double HorizontalDistance() const;
   double GetX() const { return _x; }
   double GetY() const { return _y; }
   double GetZ() const { return _z; }
   boost::math::quaternion<double> GetOrientation() const { return _orientation; }
   double GetYaw() const;
   int    GetId() const { return _id; }
};

class SwarmieSensors
{
private:
   ros::NodeHandle _nh;
   ros::Subscriber _leftSubscriber;
   ros::Subscriber _rightSubscriber;
   ros::Subscriber _centerSubscriber;
   ros::Subscriber _tagSubscriber;
   
   double _leftSonar;
   double _rightSonar;
   double _centerSonar;
   std::vector<Tag> _detections;

   void LeftSonarHandler  (const sensor_msgs::Range& range);
   void RightSonarHandler (const sensor_msgs::Range& range);
   void CenterSonarHandler(const sensor_msgs::Range& range);

   void TagHandler(const apriltags_ros::AprilTagDetectionArray::ConstPtr& message);
public:
   SwarmieSensors(std::string name);
   ~SwarmieSensors() {};
   
   double GetLeftSonar() const { return _leftSonar; }
   double GetRightSonar() const { return _rightSonar; }
   double GetCenterSonar() const { return _centerSonar; }
   const std::vector<Tag> GetTags() const { return _detections; }
};

#endif // _SWARMIE_SENSORS_HPP