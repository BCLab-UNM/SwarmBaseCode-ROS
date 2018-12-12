#include "PositionPublisher.hpp"
#include "swarmie_msgs/Recruitment.h"

#include <algorithm>

PositionPublisher::PositionPublisher(ros::NodeHandle& node_handle, std::string hostname)
{
   position_publisher = node_handle.advertise<swarmie_msgs::Recruitment>("/detectionLocations", 1);
   name.data = hostname;
}

void PositionPublisher::setDetections(std::vector<Tag> tags, Point current_position)
{
   int cube_tag_count = std::count_if(tags.begin(), tags.end(),
                                      [](Tag t) { return t.getID() == 0; });
   if(cube_tag_count > 3)
   {
      // then there is definitely more than one cube
      swarmie_msgs::Recruitment r;
      r.x = current_position.x;
      r.y = current_position.y;
      r.name = name;
      position_publisher.publish(r);
   }
}
