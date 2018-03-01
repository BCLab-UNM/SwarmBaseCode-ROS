#ifndef _ROS_SERVICE_GetGridMapInfo_h
#define _ROS_SERVICE_GetGridMapInfo_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "grid_map_msgs/GridMapInfo.h"

namespace grid_map_msgs
{

static const char GETGRIDMAPINFO[] = "grid_map_msgs/GetGridMapInfo";

  class GetGridMapInfoRequest : public ros::Msg
  {
    public:

    GetGridMapInfoRequest()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return GETGRIDMAPINFO; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GetGridMapInfoResponse : public ros::Msg
  {
    public:
      typedef grid_map_msgs::GridMapInfo _info_type;
      _info_type info;

    GetGridMapInfoResponse():
      info()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->info.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->info.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return GETGRIDMAPINFO; };
    const char * getMD5(){ return "a0be1719725f7fd7b490db4d64321ff2"; };

  };

  class GetGridMapInfo {
    public:
    typedef GetGridMapInfoRequest Request;
    typedef GetGridMapInfoResponse Response;
  };

}
#endif
