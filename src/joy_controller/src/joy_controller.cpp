#include <ros/ros.h>
#include <math.h>
#include <msgs/ThreeWheelSteerRad.h>
#include <msgs/PIDGain.h>
#include <sensor_msgs/Joy.h>
#include "ThreeWheelSteer.h"

using namespace std;

msgs::ThreeWheelSteerRad target;
msgs::PIDGain pid_gain;

// ステアのギア比
#define STEER_GEAR_RATIO 1.9

string mode = "VEHICLE";

int freq = 1000;

ThreeWheelSteer steer(32.25, 182.87);
double vx = 0.0, vy = 0.0, w = 0.0, TurnRadius = 1e6;
double v_max = 1.0, w_max = M_PI, TurnRadius_min = 0.8, TurnRadius_max = 1e6;

double Vkp[3], Vki[3], Vkd[3];
double Pkp[3], Pki[3], Pkd[3];

void joyCb(const sensor_msgs::Joy &joy_msg) {
    if (joy_msg.buttons[5]) {
        target.Stop = false;
        vy =  joy_msg.axes[0] * v_max;
        vx =  joy_msg.axes[1] * v_max;
        w  = -joy_msg.axes[2] * w_max;
        TurnRadius = -(TurnRadius_max - TurnRadius_min) * abs(joy_msg.axes[2]) + TurnRadius_max;
        ROS_INFO_STREAM(mode);

        if (mode == "VEHICLE") {
            steer.vehicle(vx, TurnRadius, w < 0);
        }
        else if (mode == "PARALLEL") {
            steer.parallel(vx, vy);
        }
        else if (mode == "ROTATE") {
            steer.rotate(w);
        }
    }
    else {
        steer.stop();
        target.Stop = true;
        ROS_INFO_STREAM("STOP");
    }

    if (joy_msg.axes[5] == 1) {
        mode = "VEHICLE";
        ROS_INFO_STREAM("MODE CHANGE: VEHCILE");
    }
    else if (joy_msg.axes[5] == -1) {
        mode = "PARALLEL";
        ROS_INFO_STREAM("MODE CHANGE: PARALLEL");
    }
    else if (joy_msg.axes[4] == 1 || joy_msg.axes[4] == -1) {
        mode = "ROTATE";
        ROS_INFO_STREAM("MODE CHANGE: ROTATE");
    }
}

void setTarget() {
    target.AngleLeft  = -steer.AngleLeft * STEER_GEAR_RATIO;
    target.AngleRight = -steer.AngleRight* STEER_GEAR_RATIO;
    target.AngleBack  = -steer.AngleBack * STEER_GEAR_RATIO;
    target.AngVelLeft  = steer.AngVelLeft;
    target.AngVelRight = steer.AngVelRight;
    target.AngVelBack  = steer.AngVelBack;
}

void setGain() {
    for (int i = 0; i < 3; i++) {
        pid_gain.Vkp[i] = Vkp[i];
        pid_gain.Vki[i] = Vki[i];
        pid_gain.Vkd[i] = Vkd[i];
        pid_gain.Pkp[i] = Pkp[i];
        pid_gain.Pki[i] = Pki[i];
        pid_gain.Pkd[i] = Pkd[i];
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "controller");
    ros::NodeHandle nh;

    ros::NodeHandle pnh("~");
    pnh.getParam("v_max", v_max);
    if(pnh.getParam("w_max", w_max)) w_max *= M_PI;
    pnh.getParam("TurnRadius_min", TurnRadius_min);
    pnh.getParam("TurnRadius_max", TurnRadius_max);
    pnh.getParam("Vkpl", Vkp[0]);
    pnh.getParam("Vkil", Vki[0]);
    pnh.getParam("Vkdl", Vkd[0]);
    pnh.getParam("Pkpl", Pkp[0]);
    pnh.getParam("Pkil", Pki[0]);
    pnh.getParam("Pkdl", Pkd[0]);
    pnh.getParam("Vkpr", Vkp[1]);
    pnh.getParam("Vkir", Vki[1]);
    pnh.getParam("Vkdr", Vkd[1]);
    pnh.getParam("Pkpr", Pkp[1]);
    pnh.getParam("Pkir", Pki[1]);
    pnh.getParam("Pkdr", Pkd[1]);
    pnh.getParam("Vkpb", Vkp[2]);
    pnh.getParam("Vkib", Vki[2]);
    pnh.getParam("Vkdb", Vkd[2]);
    pnh.getParam("Pkpb", Pkp[2]);
    pnh.getParam("Pkib", Pki[2]);
    pnh.getParam("Pkdb", Pkd[2]);
    pnh.getParam("freq", freq);

    steer.setVMax(v_max);
    steer.setWMax(w_max);
    steer.setTurnRadiusMin(TurnRadius_min);
    steer.setTurnRadiusMax(TurnRadius_max);

    target.Stop = true;

    ros::Subscriber joy_sub = nh.subscribe("joy", 1, joyCb);
    ros::Publisher target_pub = nh.advertise<msgs::ThreeWheelSteerRad>("target", 1);
    ros::Publisher gain_pub = nh.advertise<msgs::PIDGain>("gain", 1);

    sleep(5);
    setGain();
    gain_pub.publish(pid_gain);
    
    ros::Rate loop_rate(freq);
    while (ros::ok()) {
        ros::spinOnce();
        setTarget();
        target_pub.publish(target);
        loop_rate.sleep();
    }
    return 0;
}