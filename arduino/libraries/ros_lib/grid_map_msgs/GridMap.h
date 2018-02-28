#ifndef _ROS_grid_map_msgs_GridMap_h
#define _ROS_grid_map_msgs_GridMap_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "grid_map_msgs/GridMapInfo.h"
#include "std_msgs/Float32MultiArray.h"

namespace grid_map_msgs
{

  class GridMap : public ros::Msg
  {
    public:
      typedef grid_map_msgs::GridMapInfo _info_type;
      _info_type info;
      uint32_t layers_length;
      typedef char* _layers_type;
      _layers_type st_layers;
      _layers_type * layers;
      uint32_t basic_layers_length;
      typedef char* _basic_layers_type;
      _basic_layers_type st_basic_layers;
      _basic_layers_type * basic_layers;
      uint32_t data_length;
      typedef std_msgs::Float32MultiArray _data_type;
      _data_type st_data;
      _data_type * data;
      typedef uint16_t _outer_start_index_type;
      _outer_start_index_type outer_start_index;
      typedef uint16_t _inner_start_index_type;
      _inner_start_index_type inner_start_index;

    GridMap():
      info(),
      layers_length(0), layers(NULL),
      basic_layers_length(0), basic_layers(NULL),
      data_length(0), data(NULL),
      outer_start_index(0),
      inner_start_index(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->info.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->layers_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->layers_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->layers_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->layers_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->layers_length);
      for( uint32_t i = 0; i < layers_length; i++){
      uint32_t length_layersi = strlen(this->layers[i]);
      varToArr(outbuffer + offset, length_layersi);
      offset += 4;
      memcpy(outbuffer + offset, this->layers[i], length_layersi);
      offset += length_layersi;
      }
      *(outbuffer + offset + 0) = (this->basic_layers_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->basic_layers_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->basic_layers_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->basic_layers_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->basic_layers_length);
      for( uint32_t i = 0; i < basic_layers_length; i++){
      uint32_t length_basic_layersi = strlen(this->basic_layers[i]);
      varToArr(outbuffer + offset, length_basic_layersi);
      offset += 4;
      memcpy(outbuffer + offset, this->basic_layers[i], length_basic_layersi);
      offset += length_basic_layersi;
      }
      *(outbuffer + offset + 0) = (this->data_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->data_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->data_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->data_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->data_length);
      for( uint32_t i = 0; i < data_length; i++){
      offset += this->data[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->outer_start_index >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->outer_start_index >> (8 * 1)) & 0xFF;
      offset += sizeof(this->outer_start_index);
      *(outbuffer + offset + 0) = (this->inner_start_index >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->inner_start_index >> (8 * 1)) & 0xFF;
      offset += sizeof(this->inner_start_index);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->info.deserialize(inbuffer + offset);
      uint32_t layers_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      layers_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      layers_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      layers_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->layers_length);
      if(layers_lengthT > layers_length)
        this->layers = (char**)realloc(this->layers, layers_lengthT * sizeof(char*));
      layers_length = layers_lengthT;
      for( uint32_t i = 0; i < layers_length; i++){
      uint32_t length_st_layers;
      arrToVar(length_st_layers, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_layers; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_layers-1]=0;
      this->st_layers = (char *)(inbuffer + offset-1);
      offset += length_st_layers;
        memcpy( &(this->layers[i]), &(this->st_layers), sizeof(char*));
      }
      uint32_t basic_layers_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      basic_layers_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      basic_layers_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      basic_layers_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->basic_layers_length);
      if(basic_layers_lengthT > basic_layers_length)
        this->basic_layers = (char**)realloc(this->basic_layers, basic_layers_lengthT * sizeof(char*));
      basic_layers_length = basic_layers_lengthT;
      for( uint32_t i = 0; i < basic_layers_length; i++){
      uint32_t length_st_basic_layers;
      arrToVar(length_st_basic_layers, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_basic_layers; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_basic_layers-1]=0;
      this->st_basic_layers = (char *)(inbuffer + offset-1);
      offset += length_st_basic_layers;
        memcpy( &(this->basic_layers[i]), &(this->st_basic_layers), sizeof(char*));
      }
      uint32_t data_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->data_length);
      if(data_lengthT > data_length)
        this->data = (std_msgs::Float32MultiArray*)realloc(this->data, data_lengthT * sizeof(std_msgs::Float32MultiArray));
      data_length = data_lengthT;
      for( uint32_t i = 0; i < data_length; i++){
      offset += this->st_data.deserialize(inbuffer + offset);
        memcpy( &(this->data[i]), &(this->st_data), sizeof(std_msgs::Float32MultiArray));
      }
      this->outer_start_index =  ((uint16_t) (*(inbuffer + offset)));
      this->outer_start_index |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->outer_start_index);
      this->inner_start_index =  ((uint16_t) (*(inbuffer + offset)));
      this->inner_start_index |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->inner_start_index);
     return offset;
    }

    const char * getType(){ return "grid_map_msgs/GridMap"; };
    const char * getMD5(){ return "95681e052b1f73bf87b7eb984382b401"; };

  };

}
#endif