#ifndef _ROS_msgs_ThreeWheelSteerRad_h
#define _ROS_msgs_ThreeWheelSteerRad_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace msgs
{

  class ThreeWheelSteerRad : public ros::Msg
  {
    public:
      typedef float _AngleLeft_type;
      _AngleLeft_type AngleLeft;
      typedef float _AngleRight_type;
      _AngleRight_type AngleRight;
      typedef float _AngleBack_type;
      _AngleBack_type AngleBack;
      typedef float _AngVelLeft_type;
      _AngVelLeft_type AngVelLeft;
      typedef float _AngVelRight_type;
      _AngVelRight_type AngVelRight;
      typedef float _AngVelBack_type;
      _AngVelBack_type AngVelBack;
      typedef bool _Stop_type;
      _Stop_type Stop;

    ThreeWheelSteerRad():
      AngleLeft(0),
      AngleRight(0),
      AngleBack(0),
      AngVelLeft(0),
      AngVelRight(0),
      AngVelBack(0),
      Stop(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_AngleLeft;
      u_AngleLeft.real = this->AngleLeft;
      *(outbuffer + offset + 0) = (u_AngleLeft.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_AngleLeft.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_AngleLeft.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_AngleLeft.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->AngleLeft);
      union {
        float real;
        uint32_t base;
      } u_AngleRight;
      u_AngleRight.real = this->AngleRight;
      *(outbuffer + offset + 0) = (u_AngleRight.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_AngleRight.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_AngleRight.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_AngleRight.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->AngleRight);
      union {
        float real;
        uint32_t base;
      } u_AngleBack;
      u_AngleBack.real = this->AngleBack;
      *(outbuffer + offset + 0) = (u_AngleBack.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_AngleBack.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_AngleBack.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_AngleBack.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->AngleBack);
      union {
        float real;
        uint32_t base;
      } u_AngVelLeft;
      u_AngVelLeft.real = this->AngVelLeft;
      *(outbuffer + offset + 0) = (u_AngVelLeft.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_AngVelLeft.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_AngVelLeft.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_AngVelLeft.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->AngVelLeft);
      union {
        float real;
        uint32_t base;
      } u_AngVelRight;
      u_AngVelRight.real = this->AngVelRight;
      *(outbuffer + offset + 0) = (u_AngVelRight.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_AngVelRight.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_AngVelRight.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_AngVelRight.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->AngVelRight);
      union {
        float real;
        uint32_t base;
      } u_AngVelBack;
      u_AngVelBack.real = this->AngVelBack;
      *(outbuffer + offset + 0) = (u_AngVelBack.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_AngVelBack.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_AngVelBack.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_AngVelBack.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->AngVelBack);
      union {
        bool real;
        uint8_t base;
      } u_Stop;
      u_Stop.real = this->Stop;
      *(outbuffer + offset + 0) = (u_Stop.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->Stop);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_AngleLeft;
      u_AngleLeft.base = 0;
      u_AngleLeft.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_AngleLeft.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_AngleLeft.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_AngleLeft.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->AngleLeft = u_AngleLeft.real;
      offset += sizeof(this->AngleLeft);
      union {
        float real;
        uint32_t base;
      } u_AngleRight;
      u_AngleRight.base = 0;
      u_AngleRight.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_AngleRight.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_AngleRight.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_AngleRight.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->AngleRight = u_AngleRight.real;
      offset += sizeof(this->AngleRight);
      union {
        float real;
        uint32_t base;
      } u_AngleBack;
      u_AngleBack.base = 0;
      u_AngleBack.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_AngleBack.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_AngleBack.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_AngleBack.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->AngleBack = u_AngleBack.real;
      offset += sizeof(this->AngleBack);
      union {
        float real;
        uint32_t base;
      } u_AngVelLeft;
      u_AngVelLeft.base = 0;
      u_AngVelLeft.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_AngVelLeft.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_AngVelLeft.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_AngVelLeft.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->AngVelLeft = u_AngVelLeft.real;
      offset += sizeof(this->AngVelLeft);
      union {
        float real;
        uint32_t base;
      } u_AngVelRight;
      u_AngVelRight.base = 0;
      u_AngVelRight.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_AngVelRight.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_AngVelRight.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_AngVelRight.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->AngVelRight = u_AngVelRight.real;
      offset += sizeof(this->AngVelRight);
      union {
        float real;
        uint32_t base;
      } u_AngVelBack;
      u_AngVelBack.base = 0;
      u_AngVelBack.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_AngVelBack.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_AngVelBack.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_AngVelBack.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->AngVelBack = u_AngVelBack.real;
      offset += sizeof(this->AngVelBack);
      union {
        bool real;
        uint8_t base;
      } u_Stop;
      u_Stop.base = 0;
      u_Stop.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->Stop = u_Stop.real;
      offset += sizeof(this->Stop);
     return offset;
    }

    virtual const char * getType() override { return "msgs/ThreeWheelSteerRad"; };
    virtual const char * getMD5() override { return "4b38905eae57e237a2173ec2e223c93d"; };

  };

}
#endif
