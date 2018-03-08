#ifndef _ROS_SERVICE_ProcessFile_h
#define _ROS_SERVICE_ProcessFile_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace grid_map_msgs
{

static const char PROCESSFILE[] = "grid_map_msgs/ProcessFile";

  class ProcessFileRequest : public ros::Msg
  {
    public:
      typedef const char* _file_path_type;
      _file_path_type file_path;

    ProcessFileRequest():
      file_path("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_file_path = strlen(this->file_path);
      varToArr(outbuffer + offset, length_file_path);
      offset += 4;
      memcpy(outbuffer + offset, this->file_path, length_file_path);
      offset += length_file_path;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_file_path;
      arrToVar(length_file_path, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_file_path; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_file_path-1]=0;
      this->file_path = (char *)(inbuffer + offset-1);
      offset += length_file_path;
     return offset;
    }

    const char * getType(){ return PROCESSFILE; };
    const char * getMD5(){ return "a1f82596372c52a517e1fe32d1e998e8"; };

  };

  class ProcessFileResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    ProcessFileResponse():
      success(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
     return offset;
    }

    const char * getType(){ return PROCESSFILE; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class ProcessFile {
    public:
    typedef ProcessFileRequest Request;
    typedef ProcessFileResponse Response;
  };

}
#endif
