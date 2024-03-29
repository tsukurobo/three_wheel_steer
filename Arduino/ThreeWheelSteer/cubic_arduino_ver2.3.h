#ifndef Cubic_h 
#define Cubic_h
#include "Arduino.h"
#include <SPI.h>

// ENABLEをHIGHにすることによって動作開始
#define ENABLE 7

// SPI通信に用いるピン
#define MISO 12
#define MOSI 11 
#define SCK 13 
#define SS_DC_MOTOR 15  // DCモータ(ソレノイド)のSS
#define SS_INC_ENC 21   // インクリメントエンコーダのSS
#define SS_ABS_ENC 14   // アブソリュートエンコーダのSS
#define SS_ADC 20       // ADCのSS

// スレーブ側で割り込み処理を開始するためのSSの変化におけるディレイ(us)
#define SPI_DELAY 1

// モータ，エンコーダの数
#define DC_MOTOR_NUM 8
#define INC_ENC_NUM 8
#define ABS_ENC_NUM 8
#define SOL_SUB_NUM 4 // ソレノイドとサブチャンネルDCモータの数

// 送受信するデータ1つ辺りのバイト数
#define DC_MOTOR_BYTES 2
#define INC_ENC_BYTES 2
#define ABS_ENC_BYTES 2

// SPI通信におけるDCモータのDutyの最大値
#define DUTY_SPI_MAX 32766

// ソレノイドの出力を切り替える最小時間(ms)
#define SOL_TIME_MIN 10

class DC_motor {
    public:
        // 初期化する関数
        static void begin(void);
        // 指定したモータのDutyを格納する関数
        // 第1引数：モーター番号0~7，第2引数：duty，第3引数：最大duty
        static void put(uint8_t num, int16_t duty, uint16_t duty_max = 1000);
        // すべてのモータのDutyをSPI通信で送信する関数
        static void send(void);
        // すべてのモータのDutyの値をSerial.print()で表示する関数
        // ソレノイドの状態を出力している場合はSOLと表示される
        static void print(bool new_line = false);
        // RP2040への送信データを格納する配列
        // ソレノイドを使用する場合も後ろ4つの要素を使用する
        static int16_t buf[DC_MOTOR_NUM+SOL_SUB_NUM];
};

class Solenoid {
    public:
        // 初期化する関数
        static void begin(void);
        // 指定したソレノイドの状態を格納する関数
        // 第1引数：ソレノイド番号0~3，第2引数：状態
        static void put(uint8_t num, bool state);
        // 指定したソレノイドの状態を取得する関数
        static int8_t get(uint8_t num);
        // すべてのソレノイドの状態をSerial.print()で表示する関数
        static void print(bool new_line = false);
    private:
        // 状態を変更した時刻を保存する配列
        static unsigned long time_prev[SOL_SUB_NUM];
};

class Inc_enc{
    public:
        // 初期化する関数
        static void begin(void);
        // エンコーダの値の差分を取得する関数
        // 第1引数：エンコーダ番号
        static int16_t get(uint8_t num);
        // すべてのエンコーダの値の差分をSPI通信で受信する関数
        static void receive(void);
        // すべてのエンコーダの値をSerial.print()で表示する関数
        static void print(bool new_line = false);
    private:
        // RP2040からの受信データを格納する配列
        static uint8_t buf[INC_ENC_NUM*INC_ENC_BYTES];
};

class Abs_enc{
    public:
        // 初期化する関数
        static void begin(void);
        // エンコーダの値を取得する関数
        // 第1引数：エンコーダ番号
        static uint16_t get(uint8_t num);
        // すべてのエンコーダの値をSPI通信で受信する関数
        static void receive(void);
        // すべてのエンコーダの値をSerial.print()で表示する関数
        static void print(bool new_line = false);
    private:
        // RP2040からの受信データを格納する配列
        static uint8_t buf[ABS_ENC_NUM*ABS_ENC_BYTES];
};

class Cubic{
    public:
        // すべてのモータ，エンコーダの初期化をする関数
        static void begin(void);
        // データの送受信をまとめて行う関数
        static void update(void);
};

#endif