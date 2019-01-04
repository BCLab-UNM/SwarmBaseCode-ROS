#ifndef _BEHAVIOR_POSITION_PUBLISHER_HPP
#define _BEHAVIOR_POSITION_PUBLISHER_HPP

#include <vector>
#include <string>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include "Point.h"
#include "Tag.h"

class PositionPublisher
{
private:
   ros::Publisher position_publisher;
   std_msgs::String name;
   
public:
   PositionPublisher(ros::NodeHandle& node_handle, std::string hostname);
   void setDetections(std::vector<Tag> tags, Point p);
};

#endif // _BEHAVIOR_POSITION_PUBLISHER_HPP