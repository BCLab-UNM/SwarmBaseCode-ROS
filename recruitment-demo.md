# Swarmie Recruitment via ROS topics

This is a simple demo of recruitment using the [`SwarmBasecode-ROS`](https://github.com/BCLab-UNM/SwarmBaseCode-ROS)
software for the NASA Swarmathon. Recruitment is used to notify other
robots when clusters of cubes have been located to increase the number
of robots collecting from a particular pile. This demo ignores many of
the finer points of recruitment such as recruiting too many robots or
recruiting to two locations simultaneously. A fully fledged
recruitment scheme should consider these problems if it is to work
well. The basic plan is to broadcast a message with the robot&rsquo;s
current location whenever the robot can see more than one cube. Other
robots that receive the message 


## Recruitment Messages

In order to fit within the rules of the competition recruitment
messages will be sent via a ROS topic called `/detectionLocations`. To
keep things clean I will begin by defining a new message type to be
used for recruitment.

```
# swarmie_msgs/msg/Recruitment.msg
float32 x
float32 y
std_msgs/String name
```

The name of the robot is added so that robots can ignore recruitment
messages that they sent themselves. There are more canonical ways to
do this in ROS, but they won&rsquo;t work for us (we will see why
later).

We also have to modify the `CMakeLists.txt` for `swarmie_msgs` to
build the new message type by adding it to the message files list and
adding `std_msgs` as a dependency.

```cmake
find_package(catkin REQUIRED COMPONENTS 
  message_generation 
  std_msgs
)
# ...
add_message_files(
  FILES
  Waypoint.msg
  Recruitment.msg
)
#...
generate_messages(
  DEPENDENCIES
  std_msgs
)
```

## Publishing our location

To publish our location we will make a new class responsible for
deciding when to recruit and for sending the message.

```c++
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
   PositionPublisher(ros::NodeHandle& node_handle, 
                     std::string hostname);
   void setDetections(std::vector<Tag> tags, Point p);
};

#endif // _BEHAVIOR_POSITION_PUBLISHER_HPP
```

Whenever we get a message from the apriltags package with the current
detections we will pass it to an instance of this class along with the
current robot&rsquo;s current location.

```c++
void targetHandler(
   const apriltags_ros::AprilTagDetectionArray::ConstPtr& message) {
    /* ... */
    vector<Tag> tags;

    for (int i = 0; i < message->detections.size(); i++) {

      // Package up the ROS AprilTag data into our 
      // own type that does not rely on ROS.
      /* ... */
    }

    Point curr_loc;
    curr_loc.x = currentLocationMap.x;
    curr_loc.y = currentLocationMap.y;
    positionPublisher->setDetections(tags, curr_loc);

    logicController.SetAprilTags(tags);
}
```

The implementation of the `PositionPublisher` is very simple. Whenever
`PositionPublisher::setDetections()` is called it counts the number of
cube tags it sees (tag id 0) and if that number is greater than 3 it
published the current position.

```c++
#include "PositionPublisher.hpp"
#include "swarmie_msgs/Recruitment.h"

#include <algorithm> // std::count_if

PositionPublisher::PositionPublisher(ros::NodeHandle& node_handle,
                                     std::string hostname)
{
   position_publisher = 
      node_handle
      .advertise<swarmie_msgs::Recruitment>("/detectionLocations", 1);
   name.data = hostname;
}

void PositionPublisher::setDetections(std::vector<Tag> tags,
                                      Point current_position)
{
   int cube_tag_count =
      std::count_if(tags.begin(), tags.end(),
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
```

## Acting on recruitment

Add a subscriber and handler to `ROSAdapter.cpp`.

```c++
ros::Subscriber recruitmentSubscriber;

void recruitmentHandler(const swarmie_msgs::Recruitment&);

/* ... */
int main(int argc, char **argv) {
   /* ... */
   recruitmentSubscriber = mNH.subscribe("/detectionLocations",
                                         10, recruitmentHandler);
}
```

The handler will simply tell the logic controller that a recruitment
message was received after filtering out messages that were sent by
the current robot.

```c++
void recruitmentHandler(const swarmie_msgs::Recruitment& msg)
{
   if(msg.name.data != publishedName) {
      Point p;
      p.x = msg.x;
      p.y = msg.y;
      logicController.gotRecruitmentMessage(p);
   }
}
```

We need to extend the `LogicController` to act on these messages.

```c++
// Remember to add a corresponding declaration in LogicController.h
void LogicController::gotRecruitmentMessage(Point p)
{
   searchController.setRecruitmentLocation(p);
}
```

In the search controller we will simply override the previous search
location whenever a new recruitment message is received (ideally we
would want to be more selective about when we should listen to
recruitment messages).

```c++
// Remember to add a corresponding declaration in SearchController.h
void SearchController::setRecruitmentLocation(Point p)
{
   attemptCount = 1;
   // forget whatever we were doing...
   result.wpts.waypoints.clear();
   // ... and drive to p
   result.wpts.waypoints.insert(result.wpts.waypoints.begin(), p);
}
```

That&rsquo;s it! Whenever a robot receives a `Recruitment` message it
attempts to respond by driving towards the indicated location.


## Making it work with rosbridge

To make this work when robots are operating independently and
communicating with the laptop only through our rosbridge shim you need
to add some code to `misc/rosbridge/swarmie.js`. What this will do is
listen for recruitment messages being published from each robot and
pass them on to all the other robots. (All of the changes here are in
the `Swarmie` constructor.)

The first thing is to set up two roslibjs topics, one pointing at the
robot and one pointing at a complementary topic on the laptop.

```javascript
this.recruitmentTopic = new ROSLIB.Topic({
   ros : this.robotRos,
   name: "/detectionLocations",
   messageType: "swarmie_msgs/Recruitment"
});
this.ltRecruitmentTopic = new ROSLIB.Topic({
   ros : this.laptopRos,
   name: "/detectionLocations",
   messageType: "swarmie_msgs/Recruitment"
});
```

We will listen on the robot topic relaying any messages that are
tagged with the name *this robot* to the complementary laptop
topic. Simultaneously we have subscribed to the laptop version of this
topic. In this handler we do the complementary action relaying
messages from the laptop to the robot only if the message is *not*
tagged with the robot&rsquo;s name. **This selectivity is absolutely vital
because it prevents an infinite feedback loop** where the robot
publishes a message that is relayed back to itself through the laptop
at which point roslibjs sees the message and relays it again (and so
on, and so on).

```javascript
// Note that we use arrow type callbacks in order to capture
// the value of "this"!
this.recruitmentTopic.subscribe((msg) => {
   // only share our own recruitment messages
   if(msg.name.data == this.robotName)
   {
      this.ltRecruitmentTopic.publish(msg);
   }
});
this.ltRecruitmentTopic.subscribe((msg) => {
   // only pass through messages that are from other robots
   if(msg.name.data != this.robotName)
   {
      this.recruitmentTopic.publish(msg);
   }
});
```

### Benefits and Limitations

The use of our rosbridge shim above lets each robot operate
independently so they are protected from failure of the network and of
the laptop. However, if the laptop fails, recruitment will stop
working. It is possible to make recruitment robust to failure of the
laptop by setting up peer-to-peer connections between robots. This
would require

-   Node discovery. Somehow robots need to determine what other robots
    are running. You can do this in a similar way to the recruitment
    example (it is reasonable to assume that at least for a short period
    of time the laptop is up and working since that is how we start the
    robots).
-   A ROS node to set up and manage websocket connections between the
    robots. You do not need to use javascript to communicate with the
    rosbridge websocket server. This node will effectively do what the
    javascript code above does; relay messages between robots.

Note that you will need to be able to tolerate not connecting to the
websocket server in order for code to work in simulation (when the
websocket server is not running). **You should write your code (as was**
**done above) so that it is agnostic to whether messages are being sent**
**locally or via rosbridge**.

