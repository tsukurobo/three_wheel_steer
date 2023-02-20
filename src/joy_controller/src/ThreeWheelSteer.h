#ifndef ThreeWheelSteer_h
#define ThreeWheelSteer_h

#include <math.h>
#define ROOT3_2 0.86602540378443864676372317075294 // √3/2
#define M_PI_6 0.52359877559829887307710723054658 // π/6

#define FLT_ZERO 1e-5

class ThreeWheelSteer {
private:
    // 中心とホイールの距離(m)
    const double DistWheelCenter;
    // 各ホイールの速度(m/s)を角速度(rad/s)に変換するための係数
    const double VEL_TO_RAD;
    // 最大移動速度(m/s)、最大角速度(rad/s)、ただし正の値
    double v_max, w_max;
    // 最小旋回半径(m)、最大旋回半径(m)
    double TurnRadius_min, TurnRadius_max;
    // 移動速度を補正
    double vLimitter(double &vx, double &vy);
    // x軸方向の移動速度を補正
    void vxLimitter(double &vx);
    // 角速度を補正
    void wLimitter(double &w);
    // 旋回半径を補正
    bool TurnRadiusLimitter(double &TurnRadius);
    // -M_PI_2<(目標ステア角)<M_PI_2になるように目標ステア角と目標角速度を補正
    void AngleLimitter(double &Angle, double &AngVel);
public:
    // 各ホイールの目標ステア角(-M_PI_2 ~ M_PI_2)
    double AngleLeft = 0.0, AngleRight = 0.0, AngleBack = 0.0;
    // 各ホイールの目標角速度(rad/s)
    double AngVelLeft = 0.0, AngVelRight = 0.0, AngVelBack = 0.0;
    // コンストラクタ
    // WheelRadius: ホイールの半径(mm), DistWheelCenter: ホイール中心と車体中心の距離(mm)
    ThreeWheelSteer(double WheelRadius, double DistWheelCenter, double v_max = 1.0, double w_max = M_PI, double TurnRadius_min = 0.8, double TurnRadius_max = 1e6);
    // 平行移動をするための目標ステア角と目標RPSを計算
    void parallel(double vx, double vy);
    // 回転するための目標ステア角と目標RPSを計算
    void rotate(double w);
    // 自動車と同じように走行するための目標ステア角と目標RPSを計算
    void vehicle(double vx, double TurnRadius, bool TurnLeft);
    // 停止
    void stop() {
        AngVelLeft = AngVelRight = AngVelBack = 0;
    }
    // 最大移動速度を設定
    void setVMax(double v_max) {
        this->v_max = abs(v_max);
    }
    // 最大角速度を設定
    void setWMax(double w_max) {
        this->w_max = abs(w_max);
    }
    // 最小旋回半径を設定
    void setTurnRadiusMin(double TurnRadius_min) {
        this->TurnRadius_min = abs(TurnRadius_min);
    }
    // 最大旋回半径を設定
    void setTurnRadiusMax(double TurnRadius_max) {
        this->TurnRadius_max = abs(TurnRadius_max);
    }
};

#endif