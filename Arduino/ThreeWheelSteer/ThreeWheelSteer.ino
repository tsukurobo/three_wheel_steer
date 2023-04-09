#include "cubic_arduino_ver2.3.h"
#include "PID.h"
#include "Cubic.controller.h"
#include <ros.h>
#include "msgs/ThreeWheelSteerRad.h"
#include "msgs/PIDGain.h"
#include <std_msgs/Int16MultiArray.h>

#define MOTOR_NUM 3

#define FLT_ZERO 1e-5

ros::NodeHandle nh;

double AngleLeft, AngleRight, AngleBack;
double AngVelLeft, AngVelRight, AngVelBack;
double Vkp[MOTOR_NUM], Vki[MOTOR_NUM], Vkd[MOTOR_NUM];
double Pkp[MOTOR_NUM], Pki[MOTOR_NUM], Pkd[MOTOR_NUM];
const int INC_PPR = 512;
double capableDuty = 0.5;
//int16_t duty[] = {120, 153, 130};
int16_t duty[] = {220, 200, 150};
bool Stop = false;

std_msgs::Int16MultiArray duty_msg, enc_msg;

void targetCb(const msgs::ThreeWheelSteerRad &target_msg) {
    AngleLeft  = target_msg.AngleLeft;
    AngleRight = target_msg.AngleRight;
    AngleBack  = target_msg.AngleBack;
    AngVelLeft = target_msg.AngVelLeft;
    AngVelRight= target_msg.AngVelRight;
    AngVelBack = target_msg.AngVelBack;
    Stop = target_msg.Stop;
}

void gainCb(const msgs::PIDGain &gain_msg) {
    for(int i = 0; i < MOTOR_NUM; i++) {
        Vkp[i] = gain_msg.Vkp[i];
        Vki[i] = gain_msg.Vki[i];
        Vkd[i] = gain_msg.Vkd[i];
        Pkp[i] = gain_msg.Pkp[i];
        Pki[i] = gain_msg.Pki[i];
        Pkd[i] = gain_msg.Pkd[i];
    }
}

ros::Subscriber<msgs::ThreeWheelSteerRad> target_sub("target", targetCb);
ros::Subscriber<msgs::PIDGain> gain_sub("gain", gainCb);
ros::Publisher duty_pub("duty", &duty_msg);
ros::Publisher enc_pub("enc", &enc_msg);

void setup()
{
  Cubic::begin();
  
  nh.getHardware()->setBaud(115200);
  nh.initNode();

  duty_msg.data = (int16_t*)malloc(sizeof(int16_t)*6);
  duty_msg.data_length = 6;
  enc_msg.data = (int16_t*)malloc(sizeof(int16_t)*6);
  enc_msg.data_length = 6;

  nh.subscribe(target_sub);
  nh.subscribe(gain_sub);
  nh.advertise(duty_pub);
  nh.advertise(enc_pub);
}

void loop()
{
  nh.spinOnce();

  using namespace Cubic_controller;
  //static Velocity_PID VelPIDLeft (0, 2, encoderType::inc, capableDuty, VKp, VKi, VKd, AngVelLeft,  false, false, INC_PPR);
  //static Velocity_PID VelPIDRight(2, 1, encoderType::inc, capableDuty, VKp, VKi, VKd, AngVelRight, false, false, INC_PPR);
  //static Velocity_PID VelPIDBack (6, 0, encoderType::inc, capableDuty, VKp, VKi, VKd, AngVelBack,  false, false, INC_PPR);
  static Position_PID PosPIDLeft (1, 2, encoderType::abs, AMT22_PPR, capableDuty, Pkp[0], Pki[0], Pkd[0], AngleLeft,  true, false);
  static Position_PID PosPIDRight(3, 1, encoderType::abs, AMT22_PPR, capableDuty, Pkp[1], Pki[1], Pkd[1], AngleRight, true, false);
  static Position_PID PosPIDBack (7, 0, encoderType::abs, AMT22_PPR, capableDuty, Pkp[2], Pki[2], Pkd[2], AngleBack,  true, false);
  
  if (Stop) {
    for(int i = 0; i < 8; i++) {
      DC_motor::put(i, 0);
    }
  }
  else {
    DC_motor::put(0, (abs(AngVelLeft) < FLT_ZERO ? 0 : AngVelLeft > 0 ? duty[0] : -duty[0]));
    DC_motor::put(2, (abs(AngVelRight) < FLT_ZERO ? 0 : AngVelRight > 0 ? duty[1] : -duty[1]));
    DC_motor::put(6, (abs(AngVelBack) < FLT_ZERO ? 0 : AngVelBack > 0 ? duty[2] : -duty[2]));
  
    //VelPIDLeft.setTarget(AngVelLeft); VelPIDRight.setTarget(AngVelRight); VelPIDBack.setTarget(AngVelBack);
    PosPIDLeft.setTarget(AngleLeft);  PosPIDRight.setTarget(AngleRight);  PosPIDBack.setTarget(AngleBack);
    PosPIDLeft.setGains(Pkp[0], Pki[0], Pkd[0]); PosPIDRight.setGains(Pkp[1], Pki[1], Pkd[1]); PosPIDBack.setGains(Pkp[2], Pki[2], Pkd[2]);
  
    //duty_msg.data[0] = VelPIDLeft.compute(); duty_msg.data[1] = VelPIDRight.compute(); duty_msg.data[2] = VelPIDBack.compute();
    duty_msg.data[0] = AngVelLeft; duty_msg.data[1] = AngVelRight; duty_msg.data[2] = AngVelBack;
    duty_msg.data[3] = PosPIDLeft.compute(); duty_msg.data[4] = PosPIDRight.compute(); duty_msg.data[5] = PosPIDBack.compute();
    //duty_pub.publish(&duty_msg);
  
    enc_msg.data[0] = Inc_enc::get(2); enc_msg.data[1] = Inc_enc::get(1); enc_msg.data[2] = Inc_enc::get(0);
    enc_msg.data[3] = Abs_enc::get(2); enc_msg.data[4] = Abs_enc::get(1); enc_msg.data[5] = Abs_enc::get(0);
    enc_pub.publish(&enc_msg);
  
  }
  Cubic::update();
  delay(1);
}
