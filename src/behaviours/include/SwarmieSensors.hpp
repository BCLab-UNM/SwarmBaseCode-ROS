#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <apriltags_ros/AprilTagDetectionArray.h>

class Tag
{
private:
   int _id;
   double _x;
   double _y;
   double _z;
   double _yaw;
public:
   Tag(int id, double x, double y, double z, double yaw);
   ~Tag() {}
   
   double GetAlignment(double offset) const;
   double GetDistance(double offset, double height) const;
   double GetX() const { return _x; }
   double GetY() const { return _y; }
   double GetZ() const { return _z; }
   double GetYaw() const { return _yaw; }
};

class SwarmieSensors
{
private:
   ros::NodeHandle _nh;
   ros::Subscriber _leftSubscriber;
   ros::Subscriber _rightSubscriber;
   ros::Subscriber _centerSubscriber;
   ros::Subscriber _tagSubscriber;

   const double CAMERA_OFFSET = -0.023;
   const double CAMERA_HEIGHT = 0.195;
   
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
