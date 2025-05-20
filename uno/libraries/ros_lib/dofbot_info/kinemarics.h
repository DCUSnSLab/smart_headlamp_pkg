#ifndef _ROS_SERVICE_kinemarics_h
#define _ROS_SERVICE_kinemarics_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dofbot_info
{

static const char KINEMARICS[] = "dofbot_info/kinemarics";

  class kinemaricsRequest : public ros::Msg
  {
    public:
      typedef float _tar_x_type;
      _tar_x_type tar_x;
      typedef float _tar_y_type;
      _tar_y_type tar_y;
      typedef float _tar_z_type;
      _tar_z_type tar_z;
      typedef float _Roll_type;
      _Roll_type Roll;
      typedef float _Pitch_type;
      _Pitch_type Pitch;
      typedef float _Yaw_type;
      _Yaw_type Yaw;
      typedef float _cur_joint1_type;
      _cur_joint1_type cur_joint1;
      typedef float _cur_joint2_type;
      _cur_joint2_type cur_joint2;
      typedef float _cur_joint3_type;
      _cur_joint3_type cur_joint3;
      typedef float _cur_joint4_type;
      _cur_joint4_type cur_joint4;
      typedef float _cur_joint5_type;
      _cur_joint5_type cur_joint5;
      typedef float _cur_joint6_type;
      _cur_joint6_type cur_joint6;
      typedef const char* _kin_name_type;
      _kin_name_type kin_name;

    kinemaricsRequest():
      tar_x(0),
      tar_y(0),
      tar_z(0),
      Roll(0),
      Pitch(0),
      Yaw(0),
      cur_joint1(0),
      cur_joint2(0),
      cur_joint3(0),
      cur_joint4(0),
      cur_joint5(0),
      cur_joint6(0),
      kin_name("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->tar_x);
      offset += serializeAvrFloat64(outbuffer + offset, this->tar_y);
      offset += serializeAvrFloat64(outbuffer + offset, this->tar_z);
      offset += serializeAvrFloat64(outbuffer + offset, this->Roll);
      offset += serializeAvrFloat64(outbuffer + offset, this->Pitch);
      offset += serializeAvrFloat64(outbuffer + offset, this->Yaw);
      offset += serializeAvrFloat64(outbuffer + offset, this->cur_joint1);
      offset += serializeAvrFloat64(outbuffer + offset, this->cur_joint2);
      offset += serializeAvrFloat64(outbuffer + offset, this->cur_joint3);
      offset += serializeAvrFloat64(outbuffer + offset, this->cur_joint4);
      offset += serializeAvrFloat64(outbuffer + offset, this->cur_joint5);
      offset += serializeAvrFloat64(outbuffer + offset, this->cur_joint6);
      uint32_t length_kin_name = strlen(this->kin_name);
      varToArr(outbuffer + offset, length_kin_name);
      offset += 4;
      memcpy(outbuffer + offset, this->kin_name, length_kin_name);
      offset += length_kin_name;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->tar_x));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->tar_y));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->tar_z));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->Roll));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->Pitch));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->Yaw));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->cur_joint1));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->cur_joint2));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->cur_joint3));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->cur_joint4));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->cur_joint5));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->cur_joint6));
      uint32_t length_kin_name;
      arrToVar(length_kin_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_kin_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_kin_name-1]=0;
      this->kin_name = (char *)(inbuffer + offset-1);
      offset += length_kin_name;
     return offset;
    }

    virtual const char * getType() override { return KINEMARICS; };
    virtual const char * getMD5() override { return "11d857e8542c0047afc9d3b13061446f"; };

  };

  class kinemaricsResponse : public ros::Msg
  {
    public:
      typedef float _joint1_type;
      _joint1_type joint1;
      typedef float _joint2_type;
      _joint2_type joint2;
      typedef float _joint3_type;
      _joint3_type joint3;
      typedef float _joint4_type;
      _joint4_type joint4;
      typedef float _joint5_type;
      _joint5_type joint5;
      typedef float _joint6_type;
      _joint6_type joint6;
      typedef float _x_type;
      _x_type x;
      typedef float _y_type;
      _y_type y;
      typedef float _z_type;
      _z_type z;
      typedef float _Roll_type;
      _Roll_type Roll;
      typedef float _Pitch_type;
      _Pitch_type Pitch;
      typedef float _Yaw_type;
      _Yaw_type Yaw;

    kinemaricsResponse():
      joint1(0),
      joint2(0),
      joint3(0),
      joint4(0),
      joint5(0),
      joint6(0),
      x(0),
      y(0),
      z(0),
      Roll(0),
      Pitch(0),
      Yaw(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->joint1);
      offset += serializeAvrFloat64(outbuffer + offset, this->joint2);
      offset += serializeAvrFloat64(outbuffer + offset, this->joint3);
      offset += serializeAvrFloat64(outbuffer + offset, this->joint4);
      offset += serializeAvrFloat64(outbuffer + offset, this->joint5);
      offset += serializeAvrFloat64(outbuffer + offset, this->joint6);
      offset += serializeAvrFloat64(outbuffer + offset, this->x);
      offset += serializeAvrFloat64(outbuffer + offset, this->y);
      offset += serializeAvrFloat64(outbuffer + offset, this->z);
      offset += serializeAvrFloat64(outbuffer + offset, this->Roll);
      offset += serializeAvrFloat64(outbuffer + offset, this->Pitch);
      offset += serializeAvrFloat64(outbuffer + offset, this->Yaw);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->joint1));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->joint2));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->joint3));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->joint4));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->joint5));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->joint6));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->x));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->y));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->z));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->Roll));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->Pitch));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->Yaw));
     return offset;
    }

    virtual const char * getType() override { return KINEMARICS; };
    virtual const char * getMD5() override { return "906df963bc5a51f2145b13de1507f439"; };

  };

  class kinemarics {
    public:
    typedef kinemaricsRequest Request;
    typedef kinemaricsResponse Response;
  };

}
#endif
