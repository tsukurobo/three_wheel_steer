#include "ThreeWheelSteer.h"


ThreeWheelSteer::ThreeWheelSteer(double WheelRadius, double DistWheelCenter, double v_max, double w_max, double TurnRadius_min, double TurnRadius_max)
    : VEL_TO_RAD(2*M_PI / (2*abs(WheelRadius)*M_PI / 1000.0)), DistWheelCenter(abs(DistWheelCenter) / 1000.0), v_max(abs(v_max)), w_max(abs(w_max)), TurnRadius_min(abs(TurnRadius_min)), TurnRadius_max(abs(TurnRadius_max))
{
    
}

double ThreeWheelSteer::vLimitter(double &vx, double &vy) {
    // v_maxを越えていた場合、比を保ったまま補正する。
    double v = hypot(vx, vy);
    if (v > v_max) {
        vx = vx/v * v_max;
        vy = vy/v * v_max;
        return v_max;
    }
    return v;
}

void ThreeWheelSteer::vxLimitter(double &vx) {
    // v_maxを越えていた場合、補正する。
    if (abs(vx) > v_max) vx = copysign(v_max, vx);
}

void ThreeWheelSteer::wLimitter(double &w) {
    // w_maxを越えていた場合、補正する。
    if (abs(w) > w_max) w = copysign(w_max, w);
}

bool ThreeWheelSteer::TurnRadiusLimitter(double &TurnRadius) {
    bool straight = false;
    // TurnRadius_minを下回っていた場合、補正する。
    if (abs(TurnRadius) < TurnRadius_min) TurnRadius = TurnRadius_min;
    // TurnRadius_maxを上回っていた場合、補正して直進フラグを立てる
    else if (abs(TurnRadius) + FLT_ZERO > TurnRadius_max) {
        TurnRadius = TurnRadius_max;
        straight = true;
    }
    return straight;
}

void ThreeWheelSteer::AngleLimitter(double &Angle, double &AngVel) {
    // Angleが±π/2を超えていた場合、補正する。
    if (Angle > M_PI_2) {
        Angle -= M_PI;
        AngVel *= -1.0;
    }
    else if (Angle < -M_PI_2) {
        Angle += M_PI;
        AngVel *= -1.0;
    }
}

void ThreeWheelSteer::parallel(double vx, double vy) {
    double v = vLimitter(vx, vy);
    if (abs(v) < FLT_ZERO) {
        stop();
        return;
    }
    double Angle = atan2(vy, vx);
    double AngVel = v * VEL_TO_RAD;
    AngleLimitter(Angle, AngVel);
    AngleLeft = AngleRight = AngleBack = Angle;
    AngVelLeft = AngVelRight = AngVelBack = AngVel;
}

void ThreeWheelSteer::rotate(double w) {
    wLimitter(w);

    AngleLeft  = -M_PI_6;
    AngleRight =  M_PI_6;
    AngleBack  = -M_PI_2;
    double AngVel = DistWheelCenter * w * VEL_TO_RAD;
    AngVelLeft = AngVel;
    AngVelRight = AngVelBack = AngVel;
}

void ThreeWheelSteer::vehicle(double vx, double TurnRadius, bool TurnLeft) {
    TurnRadius = abs(TurnRadius);
    bool straight = TurnRadiusLimitter(TurnRadius);
    vxLimitter(vx);

    if (straight) {
        parallel(vx, 0.0);
        return;
    }

    double Speed = vx * VEL_TO_RAD;
    if (TurnLeft) {
        AngleLeft  =  atan2(DistWheelCenter/2.0, TurnRadius - ROOT3_2*DistWheelCenter);
        AngleRight =  atan2(DistWheelCenter/2.0, TurnRadius + ROOT3_2*DistWheelCenter);
        AngleBack  = -atan2(DistWheelCenter, TurnRadius);
        AngVelLeft  = hypot(TurnRadius - ROOT3_2*DistWheelCenter, DistWheelCenter/2.0)/TurnRadius * Speed;
        AngVelRight = hypot(TurnRadius + ROOT3_2*DistWheelCenter, DistWheelCenter/2.0)/TurnRadius * Speed;
        AngVelBack  = hypot(TurnRadius, DistWheelCenter)/TurnRadius * Speed;
    }
    else {
        AngleLeft  = -atan2(DistWheelCenter/2.0, TurnRadius + ROOT3_2*DistWheelCenter);
        AngleRight = -atan2(DistWheelCenter/2.0, TurnRadius - ROOT3_2*DistWheelCenter);
        AngleBack  =  atan2(DistWheelCenter, TurnRadius);
        AngVelLeft  = hypot(TurnRadius + ROOT3_2*DistWheelCenter, DistWheelCenter/2.0)/TurnRadius * Speed;
        AngVelRight = hypot(TurnRadius - ROOT3_2*DistWheelCenter, DistWheelCenter/2.0)/TurnRadius * Speed;
        AngVelBack  = hypot(TurnRadius, DistWheelCenter)/TurnRadius * Speed;
    }
    // 旋回半径が小さすぎない限り、±π/2を超えることはないが、念の為補正する。
    AngleLimitter(AngleLeft, AngVelLeft);
    AngleLimitter(AngleRight, AngVelRight);
    AngleLimitter(AngleBack, AngVelBack);
}