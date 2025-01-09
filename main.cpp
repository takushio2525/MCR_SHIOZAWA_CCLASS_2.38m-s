//------------------------------------------------------------------//
// Supported MCU:   RZ/A1H
// File Contents:   kit_kirokukai2021_gr_peach ｸﾛｽﾗｲﾝ,左右ﾊｰﾌﾗｲﾝあり版
// Version number:  Ver.1.00
// Date:            2020.09.08
// Copyright:       Renesas Electronics Corporation
//                 Hitachi Document Solutions Co., Ltd.
//                 宮崎県高等学校教育研究会　工業部会
//                 　　　マイコンカーラリー指導担当代表者会
//------------------------------------------------------------------//

//------------------------------------------------------------------//
// Include
//------------------------------------------------------------------//
#include "DisplayBace.h"
#include "Encoder.hpp"
#include "SdUsbConnect.h"
#include "UnbufferedSerial.h"
#include "image_process.h"
#include "iodefine.h"
#include "mbed.h"
#include <cstdio>
//------------------------------------------------------------------//
// Define
//------------------------------------------------------------------//
// Motor PWM cycle
#define MOTOR_PWM_CYCLE \
    33332 // Motor PWM period
          // 1ms    P0φ/1  = 0.03us
// Servo PWM cycle
#define SERVO_PWM_CYCLE \
    33332 // SERVO PWM period
          // 16ms   P0φ/16 = 0.48us
#define SERVO_CENTER \
    3090 // 3090        // 1.5ms / 0.48us - 1 = 3124   最初３０５０2950
// 値を足すと右　減らすと左
// 3100
#define HANDLE_STEP 23 // 1 degree value

// #define LANE_HANDLE_CALK encoder.getCnt() * lanePowerGain + laneHandleVal

// #define LANE_COUNTER_HANDLE_CALK encoder.getCnt() * 0.7 + 12

#define LANE_HANDLE_CALK 26

#define LANE_COUNTER_HANDLE_CALK 28

// #define LANE_DISTANCE laneDistance = map(encoder.getCnt(), 40, 50, 280, 350);
// #define LANE_COUNTER_DISTANCE laneCounterDistance = map(encoder.getCnt(), 40, 50, 350, 330);

#define LANE_DISTANCE laneDistance = 340;
#define LANE_COUNTER_DISTANCE laneCounterDistance = 290;

//------------------------------------------------------------------//
// マスク値設定 ×：マスクあり(無効)　○：マスク無し(有効)
//------------------------------------------------------------------//
#define MASK2_2 0x66 // ×○○××○○×
#define MASK2_0 0x60 // ×○○×××××
#define MASK0_2 0x06 // ×××××○○×
#define MASK3_3 0xe7 // ○○○××○○○
#define MASK0_3 0x07 // ×××××○○○
#define MASK3_0 0xe0 // ○○○×××××
#define MASK4_0 0xf0 // ○○○○××××
#define MASK0_4 0x0f // ××××○○○○
#define MASK4_4 0xff // ○○○○○○○○

//------------------------------------------------------------------//
// Define(NTSC-Video)
//------------------------------------------------------------------//
#define VIDEO_INPUT_CH (DisplayBase::VIDEO_INPUT_CHANNEL_0)
#define VIDEO_INT_TYPE (DisplayBase::INT_TYPE_S0_VFIELD)
#define DATA_SIZE_PER_PIC (2u)

//! Frame buffer stride: Frame buffer stride should be set to a multiple of 32 or 128
//  in accordance with the frame buffer burst transfer mode.
#define PIXEL_HW (160u) // QVGA
#define PIXEL_VW (120u) // QVGA
#define VIDEO_BUFFER_STRIDE (((PIXEL_HW * DATA_SIZE_PER_PIC) + 31u) & ~31u)
#define VIDEO_BUFFER_HEIGHT (PIXEL_VW)

//------------------------------------------------------------------//
// ここから自作defineを追加
//------------------------------------------------------------------//

#define LOG_NUM (4000u) // ログ保存数

#define IMAGE_WIDTH 160                    // 画像のX列数
#define IMAGE_HEIGHT 120                   // 画像のY行数
#define IMAGE_CENTER IMAGE_WIDTH / 2       // 画像の中心
#define IMAGE_RIGHT_EDGE IMAGE_WIDTH - 1   // 画像の右端
#define IMAGE_BOTTOM_EDGE IMAGE_HEIGHT - 1 // 画像の下端

#define MAX_MOTOR_POWER 100

#define ONEMETOR_PULSE 1020

//------------------------------------------------------------------//
// Constructor
//------------------------------------------------------------------//
// Create DisplayBase object
DisplayBase Display;

Ticker interrput;
UnbufferedSerial pc(USBTX, USBRX);

DigitalOut LED_R(P6_13);     // LED1 on the GR-PEACH board
DigitalOut LED_G(P6_14);     // LED2 on the GR-PEACH board
DigitalOut LED_B(P6_15);     // LED3 on the GR-PEACH board
DigitalOut USER_LED(P6_12);  // USER_LED on the GR-PEACH board
DigitalIn user_botton(P6_0); // SW1 on the GR-PEACH board

BusIn dipsw(P7_15, P8_1, P2_9, P2_10); // SW1 on Shield board

DigitalOut Left_motor_signal(P4_6);  // Used by motor function
DigitalOut Right_motor_signal(P4_7); // Used by motor function
DigitalIn push_sw(P2_13);            // SW1 on the Motor Drive board
DigitalOut LED_3(P2_14);             // LED3 on the Motor Drive board
DigitalOut LED_2(P2_15);             // LED2 on the Motor Drive board

Encoder encoder;

//------------------------------------------------------------------//
// Prototype
//------------------------------------------------------------------//
void init_Camera(void);
void ChangeFrameBuffer(void);
static void IntCallbackFunc_Vfield(DisplayBase::int_type_t int_type);
static void WaitVfield(const int32_t wait_count);
static void IntCallbackFunc_Vsync(DisplayBase::int_type_t int_type);
static void WaitVsync(const int32_t wait_count);
void init_MTU2_PWM_Motor(void); // Initialize PWM functions
void init_MTU2_PWM_Servo(void); // Initialize PWM functions
void intTimer(void);            // 1ms period
unsigned char user_button_get(void);
void led_m(int time, int r, int g, int b);
void led_m_process(void); // Only function for interrupt

unsigned char sensor_inp(unsigned char mask);
int check_crossline(void);
int check_rightline(void);
int check_leftline(void);
void led_out(int led);
unsigned char pushsw_get(void);
void motor(int accele_l, int accele_r);
void handle(int angle);
unsigned char dipsw_get(void);
unsigned char shikiichi_henkan(int gyou, int s, int sa);
char getImage(int ix, int iy);

int getCompileYear(const char *p);
int getCompileMonth(const char *p);
int getCompileDay(const char *p);
int getCompileHour(const char *p);
int getCompilerMinute(const char *p);
int getCompilerSecond(const char *p);
unsigned long convertBCD_CharToLong(unsigned char hex);

//------------------------------------------------------------------//
// ここから自作関数のプロトタイプ宣言を追加
//------------------------------------------------------------------//
void easyCreateDeviation(int rowNum);
void createLineFlag(int rowNum, int height); // ラインを検出する行数); // ラインフラグを生成する関数　　    switch (counter++)内で画像更新につき一回ごと実行
void createDeviation(void);

void createHandleVal(void);
void createMotorVal(void);
void createBrakeMotorVal(int targetSpeed);

long map(long x, long in_min, long in_max, long out_min, long out_max);

//------------------------------------------------------------------//
// Global variable (NTSC-video)
//------------------------------------------------------------------//
static uint8_t FrameBuffer_Video_A[VIDEO_BUFFER_STRIDE * VIDEO_BUFFER_HEIGHT] __attribute((section("NC_BSS"), aligned(16))); // 16 bytes aligned!;
static uint8_t FrameBuffer_Video_B[VIDEO_BUFFER_STRIDE * VIDEO_BUFFER_HEIGHT] __attribute((section("NC_BSS"), aligned(16))); // 16 bytes aligned!;
uint8_t *write_buff_addr = FrameBuffer_Video_A;
uint8_t *save_buff_addr = FrameBuffer_Video_B;
static volatile int32_t vsync_count;
static volatile int32_t vfield_count;
static volatile int32_t vfield_count2 = 1;
static volatile int32_t vfield_count2_buff;

//------------------------------------------------------------------//
// Global variable for Image process
//------------------------------------------------------------------//
unsigned char ImageData_A[((PIXEL_HW * 2) * PIXEL_VW)];
unsigned char ImageData_B[(PIXEL_HW * PIXEL_VW)];
unsigned char ImageComp_B[(PIXEL_HW * PIXEL_VW)];
unsigned char ImageBinary[(PIXEL_HW * PIXEL_VW)];

//------------------------------------------------------------------//
// Global variable for Trace program
//------------------------------------------------------------------//
const char *C_DATE = __DATE__; // コンパイルした日付
const char *C_TIME = __TIME__; // コンパイルした時間
const char monthStr[] = {"JanFebMarAprMayJunJulAugSepOctNovDec"};
// 月変換テーブル

volatile unsigned long cnt1; // Used within main
volatile unsigned long cnt_printf;
volatile unsigned long cnt_msd;
volatile unsigned long cnt_msdwritetime;
volatile unsigned long cnt_debug;

volatile int pattern;      // Pattern numbers
volatile int initFlag = 1; // Initialize flag

volatile int mled_time, mled_r, mled_g, mled_b;
volatile int debug_mode;

FILE *fp;
struct tm t; // microSDの日付用
volatile unsigned char sensor_bin;
volatile unsigned char bar;
volatile int log_pattern = 901;
volatile int log_mode; // 0:待機 1:ファイルオープン
                       // 2:ログ記録中 3:ログファイルクローズ
                       // ※クローズしないとファイルが保存されない
volatile int msdError;
volatile int msd_handle, msd_l, msd_r;

//------------------------------------------------------------------//
// ここから自作のグローバル変数を追加
//------------------------------------------------------------------//

volatile signed int deviationDifference;

volatile signed int trace_offset = 0;

volatile int easyImageData[IMAGE_WIDTH];
volatile int easyDifference[IMAGE_WIDTH];
volatile int easyDeviation;
volatile int easyDifferencePoint[IMAGE_WIDTH];

volatile bool endflag = false;

volatile bool lineflag_center = false;
volatile bool lineflag_left = false;
volatile bool lineflag_right = false;
volatile bool lineflag_cross = false;

volatile int shioCheck = 100;

volatile signed int allDeviation[IMAGE_HEIGHT];

volatile signed int leftDeviation[IMAGE_WIDTH];
volatile signed int rightDeviation[IMAGE_WIDTH];
volatile signed int difference[IMAGE_HEIGHT][IMAGE_WIDTH];

volatile signed int encoderAcceleration;
volatile signed int lastEncoderSpeed;

volatile int handleVal;

volatile int leftMotor;
volatile int rightMotor;

volatile int leftBrakeMotor;
volatile int rightBrakeMotor;

volatile float crankMotorPowerIN;
volatile float crankMotorPowerOUT;
volatile float crankHandleVal;

volatile float constCrankHandleVal;
volatile float constCrankMotorPowerIN;
volatile float constCrankMotorPowerOUT;

volatile float crankMotorPowerINGain;
volatile float crankMotorPowerOUTGain;
volatile float crankHandleValGain;

volatile float lanePowerGain;

volatile int laneMotorPowerIN;
volatile int laneMotorPowerOUT;
volatile int laneHandleVal;

volatile int laneCounterMotorPowerIN;
volatile int laneCounterMotorPowerOUT;
volatile int laneCounterHandleVal;

volatile int laneStraightMotorPower;

volatile int lineSkipDistance;

volatile int crankDistance;
volatile int laneDistance;
volatile int laneAfterDistance;
volatile int laneCounterDistance;
volatile bool resetFlag = true;

volatile int lowSpeedLimit;
volatile int lowSpeedCrankHandle;
volatile int lowSpeedCrankDistance;

volatile int lowSpeedLaneHandle;
volatile int lowSpeedLaneDistance;

volatile int flagLine = 0;
volatile int flagLineNomal = 0;

typedef struct
{
    unsigned int cnt_msdwritetime;
    unsigned int pattern;
    unsigned int convertBCD;
    signed int handle;
    signed int hennsa;
    unsigned int encoder;
    signed int motorL;
    signed int motorR;
    signed int flagL;
    signed int flagK;
    signed int flagR;
    signed int flagS;
    signed int total;

    // int senn(int linegyou)関数内で直接取得
    // ログ取得用(生データ)
    signed int imageData0[160]; // 白線検出時に使用する生データ
    // signed int imageData1[160]; // 白線検出時に使用する生データ

    // signed int imageData2[160]; // 白線検出時に使用する生データ

} SDLOG_T;

SDLOG_T log_data[LOG_NUM]; // ログ保存データ
unsigned int log_no = 0;   // ログの記録番地（時間）

///****************************************************************
// Main function
///****************************************************************
int main(void)
{
    int x, y, i;
    char moji_work[128], c;
    int sw_now = 0, sw_before = -1;

    initFlag = 1; // Initialization start

    // 起動時にボタンが押されているとデバッグモードON
    if (user_button_get() != 0)
    {
        debug_mode = 1;
    }

    // Camera start
    init_Camera();
    // wait to stabilize NTSC signal (about 170ms)
    ThisThread::sleep_for(200ms);

    // Initialize MCU functions
    init_MTU2_PWM_Motor();
    init_MTU2_PWM_Servo();
    encoder.init();
    interrput.attach(&intTimer, 1ms);
    pc.baud(230400);
    printf("\033[H"); // デバッグ用画面クリア
    printf("\033[2J");

    // Initialize Micon Car state
    handle(0);
    motor(0, 0);
    led_out(0x0);
    led_m(10, 0, 1, 1); // 10% R=0,G=1,B=1

    t.tm_sec = getCompilerSecond(C_TIME);      // 0-59
    t.tm_min = getCompilerMinute(C_TIME);      // 0-59
    t.tm_hour = getCompileHour(C_TIME);        // 0-23
    t.tm_mday = getCompileDay(C_DATE);         // 1-31
    t.tm_mon = getCompileMonth(C_DATE) - 1;    // 0-11 +1する
    t.tm_year = getCompileYear(C_DATE) - 1900; // -1900する
    time_t seconds = mktime(&t);
    set_time(seconds);

    SdUsbConnect storage("storage");
    // storage.wait_connect(); // 認識するまでここで止まってしまうのでこの命令は使えない
    cnt_msd = 0;
    while (storage.connect() == 0)
    { // STORAGE_NON = 0
        ThisThread::sleep_for(100ms);
        if (cnt_msd >= 1000)
        { // この時間以上microSDの接続が確認できなければエラー
            msdError = 1;
            break;
        }
    }
    if (msdError == 1)
    {
        led_m(50, 1, 0, 0);
        cnt_msd = 0;
        while (cnt_msd < 2000)
        { // 2s待つ
        }
    }

    initFlag = 0; // Initialization end

    // Debug Program
    if (debug_mode == 1)
    {
        led_m(10, 0, 1, 1);

        while (1)
        {
            // LEDにモニターする
            led_out((sensor_inp(0x80) >> 6) | sensor_inp(0x01));
            USER_LED = sensor_inp(0x18) != 0x00 ? 1 : 0;

            // ユーザーボタンを(長めに)押すとデバッグ画面を切り換える
            if (cnt_debug >= 10)
            {
                cnt_debug = 0;
                sw_now = user_button_get();
                if (sw_now == 1 && sw_before == 0)
                {
                    debug_mode++;
                    if (debug_mode >= 5)
                    {
                        debug_mode = 1;
                    }
                    printf("\033[H");
                    printf("\033[2J"); // 画面クリア
                }
                sw_before = sw_now;
            }

            if (cnt_printf >= 200)
            {
                cnt_printf = 0;

                switch (debug_mode)
                {
                case 1:
                    // 補正値で表示 しきい値180以上を"1" 180を変えると、しきい値が変わる
                    // printf("shikii chi 180\r\n");
                    // for (y = 0; y < 30; y++)
                    // {
                    //     printf("%3d:%08ld ", y + 0, convertBCD_CharToLong(shikiichi_henkan(y + 0, 180, 8)));
                    //     printf("%3d:%08ld ", y + 30, convertBCD_CharToLong(shikiichi_henkan(y + 30, 180, 8)));
                    //     printf("%3d:%08ld ", y + 60, convertBCD_CharToLong(shikiichi_henkan(y + 60, 180, 8)));
                    //     printf("%3d:%08ld ", y + 90, convertBCD_CharToLong(shikiichi_henkan(y + 90, 180, 8)));
                    //     printf("\r\n");
                    // }
                    for (x = 0; x < 160; x++)
                    {
                        c = difference[60][x] < -7 ? 1 : 0; // 180を変えるとしきい値が変わる
                        if (difference[60][x] < -7)
                        {
                            printf("\x1b[44m%d\x1b[49m", c);
                        }
                        else
                        {
                            printf("%d", c);
                        }
                    }
                    printf("\033[H");
                    break;

                case 2:
                    // 補正値で表示 しきい値120以上を"1" 180を変えると、しきい値が変わる
                    printf("shikii chi 120\r\n");
                    for (y = 0; y < 30; y++)
                    {
                        printf("%3d:%08ld ", y + 0, convertBCD_CharToLong(shikiichi_henkan(y + 0, 120, 8)));
                        printf("%3d:%08ld ", y + 30, convertBCD_CharToLong(shikiichi_henkan(y + 30, 120, 8)));
                        printf("%3d:%08ld ", y + 60, convertBCD_CharToLong(shikiichi_henkan(y + 60, 120, 8)));
                        printf("%3d:%08ld ", y + 90, convertBCD_CharToLong(shikiichi_henkan(y + 90, 120, 8)));
                        printf("\r\n");
                    }
                    printf("\033[H");
                    break;

                case 3:
                    // https://www.sejuku.net/blog/24934
                    // 文字の色
                    // \x1b[30m 黒 \x1b[31m 赤 \x1b[32m 緑 \x1b[33m 黄
                    // \x1b[34m 青 \x1b[35m マゼンダ \x1b[36m シアン
                    // \x1b[37m 白 \x1b[39m デフォルトに戻す
                    // 背景色の指定
                    // \x1b[40m 黒 \x1b[41m 赤 \x1b[42m 緑 \x1b[43m 黄
                    // \x1b[44m 青 \x1b[45m マゼンダ \x1b[46m シアン
                    // \x1b[47m 灰 \x1b[49m デフォルトに戻す

                    // 1行飛ばしで表示(しきい値180以上を"1"とする)
                    printf("shi 0         0         0         0         0         0         0         0         0         0         1         1         1         1         1         1        1\r\n");
                    printf("kii 0         1         2         3         4         5         6         7         8         9         0         1         2         3         4         5        5\r\n");
                    printf("180 0123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789\r\n");
                    for (y = 30; y < 100; y += 2)
                    {
                        printf("%03d:", y);
                        for (x = 0; x < 160; x++)
                        {
                            c = getImage(x, y) >= 200 ? 1 : 0; // 180を変えるとしきい値が変わる
                            if (x == -allDeviation[y] + 80)
                            {
                                printf("\x1b[43m%d\x1b[49m", c);
                            }
                            else if (x == -leftDeviation[y] + 80)
                            {
                                printf("\x1b[44m%d\x1b[49m", c);
                            }
                            else if (x == -rightDeviation[y] + 80)
                            {
                                printf("\x1b[41m%d\x1b[49m", c);
                            }
                            else
                            {
                                printf("%d", c);
                            }
                        }
                        printf("  \r\n");
                    }
                    printf("\033[H");
                    break;

                case 4:
                    // 60～119行を表示(しきい値120以上を"1"とする)
                    printf("shi 0         0         0         0         0         0         0         0         0         0         1         1         1         1         1         1        1\r\n");
                    printf("kii 0         1         2         3         4         5         6         7         8         9         0         1         2         3         4         5        5\r\n");
                    printf("120 0123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789\r\n");
                    for (y = 60; y < 120; y++)
                    {
                        printf("%03d:", y);
                        for (x = 0; x < 160; x++)
                        {
                            c = getImage(x, y) >= 120 ? 1 : 0; // 120を変えるとしきい値が変わる
                            if ((y == 110 || y == 111 || y == 112 || y == 113 || y == 114) && (x == 31 || x == 128))
                            {
                                printf("\x1b[43m%d\x1b[49m", c);
                            }
                            else
                            {
                                printf("%d", c);
                            }
                        }
                        printf("  \r\n");
                    }
                    printf("\033[H");
                    break;
                }
            }
        }
    }

    // 通常走行
    // ターミナルに出力とmicroSDへログ保存
    // 走行プログラムは、intTimer関数(1msごとの割り込み)で行う
    while (1)
    {

        // ログ保存処理
        if (endflag == true)
        {
            led_m(50, 1, 0, 0); // スタートバーセットOK状態→緑色点灯

            i = 0;
            if ((fp = fopen("/storage/renban.txt", "r")) != NULL)
            {
                fscanf(fp, "%d", &i);
                fclose(fp);
            }
            if (i < 0 || i > 9999)
            {
                i = 0;
            }
            if ((fp = fopen("/storage/renban.txt", "w")) != NULL)
            {
                fprintf(fp, "%d", i + 1);
                fclose(fp);
            }
            sprintf(moji_work, "/storage/data%04d.csv", i);
            if ((fp = fopen(moji_work, "w")) != NULL)
            {
                fprintf(fp, "Log %d\n", i);
            }

            for (unsigned int i = 0; i < log_no; i++)
            {
                fprintf(fp, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                        log_data[i].cnt_msdwritetime,
                        log_data[i].pattern,
                        log_data[i].convertBCD,
                        log_data[i].handle,
                        log_data[i].hennsa,
                        log_data[i].encoder,
                        log_data[i].motorL,
                        log_data[i].motorR,
                        log_data[i].flagL,
                        log_data[i].flagK,
                        log_data[i].flagR,
                        log_data[i].flagS,
                        log_data[i].total,
                        i);

                // ログ取得用(生データ)
                // for (int u = 0; u < 3; u++)
                // {
                for (int t = 0; t < 160; t++)
                {
                    fprintf(fp, "%d,", log_data[i].imageData0[t]);
                }

                fprintf(fp, "\n");

                // }
            }
            //         // microSDに保存する内容
            //         // 文字数に制限はないが、増やしすぎると10msごとにならない
            // //        fprintf(fp, "%d,%d,b%08ld,%d,%d,%x,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
            // //                int(cnt_msdwritetime % 1000), pattern,
            // //                convertBCD_CharToLong(sensor_inp(0xff)), msd_handle, ALL_hennsa[45],
            // //                encoder.getCnt(), msd_l, msd_r);
            // //    }
            fclose(fp); // fcloseしないと保存されない
            while (true)
            {
                led_m(50, 0, 0, 1); // スタートバーセットOK状態→緑色点灯
            }
        }
    }
}

///****************************************************************
// Initialize functions
///****************************************************************
//------------------------------------------------------------------//
// Initialize MTU2 PWM functions
//------------------------------------------------------------------//
// MTU2_3, MTU2_4
// Reset-Synchronized PWM mode
// TIOC4A(P4_4) :Left-motor
// TIOC4B(P4_5) :Right-motor
//------------------------------------------------------------------//
void init_MTU2_PWM_Motor(void)
{
    // Port setting for S/W I/O Control
    // alternative mode

    // MTU2_4 (P4_4)(P4_5)
    GPIOPBDC4 = 0x0000;   // Bidirection mode disabled
    GPIOPFCAE4 &= 0xffcf; // The alternative function of a pin
    GPIOPFCE4 |= 0x0030;  // The alternative function of a pin
    GPIOPFC4 &= 0xffcf;   // The alternative function of a pin
                          // 2nd altemative function/output
    GPIOP4 &= 0xffcf;     //
    GPIOPM4 &= 0xffcf;    // p4_4,P4_5:output
    GPIOPMC4 |= 0x0030;   // P4_4,P4_5:double

    // Module stop 33(MTU2) canceling
    CPGSTBCR3 &= 0xf7;

    // MTU2_3 and MTU2_4 (Motor PWM)
    MTU2TCR_3 = 0x20;            // TCNT Clear(TGRA), P0φ/1
    MTU2TOCR1 = 0x04;            //
    MTU2TOCR2 = 0x40;            // N L>H  P H>L
    MTU2TMDR_3 = 0x38;           // Buff:ON Reset-Synchronized PWM mode
    MTU2TMDR_4 = 0x30;           // Buff:ON
    MTU2TOER = 0xc6;             // TIOC3B,4A,4B enabled output
    MTU2TCNT_3 = MTU2TCNT_4 = 0; // TCNT3,TCNT4 Set 0
    MTU2TGRA_3 = MTU2TGRC_3 = MOTOR_PWM_CYCLE;
    // PWM-Cycle(1ms)
    MTU2TGRA_4 = MTU2TGRC_4 = 0; // Left-motor(P4_4)
    MTU2TGRB_4 = MTU2TGRD_4 = 0; // Right-motor(P4_5)
    MTU2TSTR |= 0x40;            // TCNT_4 Start
}

//------------------------------------------------------------------//
// Initialize MTU2 PWM functions
//------------------------------------------------------------------//
// MTU2_0
// PWM mode 1
// TIOC0A(P4_0) :Servo-motor
//------------------------------------------------------------------//
void init_MTU2_PWM_Servo(void)
{
    // Port setting for S/W I/O Control
    // alternative mode

    // MTU2_0 (P4_0)
    GPIOPBDC4 = 0x0000;   // Bidirection mode disabled
    GPIOPFCAE4 &= 0xfffe; // The alternative function of a pin
    GPIOPFCE4 &= 0xfffe;  // The alternative function of a pin
    GPIOPFC4 |= 0x0001;   // The alternative function of a pin
                          // 2nd alternative function/output
    GPIOP4 &= 0xfffe;     //
    GPIOPM4 &= 0xfffe;    // p4_0:output
    GPIOPMC4 |= 0x0001;   // P4_0:double

    // Module stop 33(MTU2) canceling
    CPGSTBCR3 &= 0xf7;

    // MTU2_0 (Motor PWM)
    MTU2TCR_0 = 0x02;            // 22 // TCNT Clear(TGRA), P0φ/16 henkou
    MTU2TIORH_0 = 0x52;          // TGRA L>H, TGRB H>L
    MTU2TMDR_0 = 0x32;           // TGRC and TGRD = Buff-mode
                                 // PWM-mode1
    MTU2TBTM_0 = 0x03;           // バッファ動作転送は TCNT クリア時 tuika
    MTU2TCNT_0 = 0;              // TCNT0 Set 0
                                 // MTU2TGRA_0 = MTU2TGRC_0 = SERVO_PWM_CYCLE;
    MTU2TGRA_0 = MTU2TGRC_0 = 2; // SV パルス立ち上がり
    // PWM-Cycle(16ms)
    // MTU2TGRB_0 = MTU2TGRD_0 = 0; // Servo-motor(P4_0)
    MTU2TGRB_0 = MTU2TGRD_0 = SERVO_CENTER; // SV パルス立ち下がり ※３

    MTU2TSTR |= 0x01; // TCNT_0 Start
}

//------------------------------------------------------------------//
// Initialize Camera function
//------------------------------------------------------------------//
void init_Camera(void)
{
    // NTSC-Video
    DisplayBase::graphics_error_t error;

    // Graphics initialization process
    error = Display.Graphics_init(NULL);
    if (error != DisplayBase::GRAPHICS_OK)
    {
        printf("Line %d, error %d\n", __LINE__, error);
        while (1)
            ;
    }

    error = Display.Graphics_Video_init(DisplayBase::INPUT_SEL_VDEC, NULL);
    if (error != DisplayBase::GRAPHICS_OK)
    {
        while (1)
            ;
    }

    // Interrupt callback function setting (Vsync signal input to scaler 0)
    error = Display.Graphics_Irq_Handler_Set(DisplayBase::INT_TYPE_S0_VI_VSYNC, 0, IntCallbackFunc_Vsync);
    if (error != DisplayBase::GRAPHICS_OK)
    {
        printf("Line %d, error %d\n", __LINE__, error);
        while (1)
            ;
    }

    // Video capture setting (progressive form fixed)
    error = Display.Video_Write_Setting(
        VIDEO_INPUT_CH,
        DisplayBase::COL_SYS_NTSC_358,
        write_buff_addr,
        VIDEO_BUFFER_STRIDE,
        DisplayBase::VIDEO_FORMAT_YCBCR422,
        DisplayBase::WR_RD_WRSWA_32_16BIT,
        PIXEL_VW,
        PIXEL_HW);
    if (error != DisplayBase::GRAPHICS_OK)
    {
        printf("Line %d, error %d\n", __LINE__, error);
        while (1)
            ;
    }

    // Interrupt callback function setting (Field end signal for recording function in scaler 0)
    error = Display.Graphics_Irq_Handler_Set(VIDEO_INT_TYPE, 0, IntCallbackFunc_Vfield);
    if (error != DisplayBase::GRAPHICS_OK)
    {
        printf("Line %d, error %d\n", __LINE__, error);
        while (1)
            ;
    }

    // Video write process start
    error = Display.Video_Start(VIDEO_INPUT_CH);
    if (error != DisplayBase::GRAPHICS_OK)
    {
        printf("Line %d, error %d\n", __LINE__, error);
        while (1)
            ;
    }

    // Video write process stop
    error = Display.Video_Stop(VIDEO_INPUT_CH);
    if (error != DisplayBase::GRAPHICS_OK)
    {
        printf("Line %d, error %d\n", __LINE__, error);
        while (1)
            ;
    }

    // Video write process start
    error = Display.Video_Start(VIDEO_INPUT_CH);
    if (error != DisplayBase::GRAPHICS_OK)
    {
        printf("Line %d, error %d\n", __LINE__, error);
        while (1)
            ;
    }

    // Wait vsync to update resister
    WaitVsync(1);

    // Wait 2 Vfield(Top or bottom field)
    WaitVfield(2);
}

//------------------------------------------------------------------//
// ChangeFrameBuffer function
//------------------------------------------------------------------//
void ChangeFrameBuffer(void)
{
    // NTSC-Video
    DisplayBase::graphics_error_t error;

    // Change write buffer
    if (write_buff_addr == FrameBuffer_Video_A)
    {
        write_buff_addr = FrameBuffer_Video_B;
        save_buff_addr = FrameBuffer_Video_A;
    }
    else
    {
        write_buff_addr = FrameBuffer_Video_A;
        save_buff_addr = FrameBuffer_Video_B;
    }
    error = Display.Video_Write_Change(
        VIDEO_INPUT_CH,
        write_buff_addr,
        VIDEO_BUFFER_STRIDE);
    if (error != DisplayBase::GRAPHICS_OK)
    {
        printf("Line %d, error %d\n", __LINE__, error);
        while (1)
            ;
    }
}

//------------------------------------------------------------------//
// @brief       Interrupt callback function
// @param[in]   int_type    : VDC5 interrupt type
// @retval      None
//------------------------------------------------------------------//
static void IntCallbackFunc_Vfield(DisplayBase::int_type_t int_type)
{
    (void)int_type;

    if (vfield_count > 0)
    {
        vfield_count--;
    }
    // top or bottom (Change)
    if (vfield_count2 == 0)
        vfield_count2 = 1;
    else
        vfield_count2 = 0;
}

//------------------------------------------------------------------//
// @brief       Wait for the specified number of times Vsync occurs
// @param[in]   wait_count          : Wait count
// @retval      None
//------------------------------------------------------------------//
static void WaitVfield(const int32_t wait_count)
{
    vfield_count = wait_count;
    while (vfield_count > 0)
    {
        // Do nothing
    }
}

//------------------------------------------------------------------//
// @brief       Interrupt callback function for Vsync interruption
// @param[in]   int_type    : VDC5 interrupt type
// @retval      None
//------------------------------------------------------------------//
static void IntCallbackFunc_Vsync(DisplayBase::int_type_t int_type)
{
    (void)int_type;

    if (vsync_count > 0)
    {
        vsync_count--;
    }
}

//------------------------------------------------------------------//
// @brief       Wait for the specified number of times Vsync occurs
// @param[in]   wait_count          : Wait count
// @retval      None
//------------------------------------------------------------------//
static void WaitVsync(const int32_t wait_count)
{
    vsync_count = wait_count;
    while (vsync_count > 0)
    {
        // Do nothing
    }
}

//------------------------------------------------------------------//
// Interrupt function( intTimer )
//------------------------------------------------------------------//
void intTimer(void)
{
    static int counter = 0; // Only variable for image process
    unsigned char b;
    int lineHeight;
    cnt1++;
    cnt_msdwritetime++;
    cnt_printf++;
    cnt_msd++;
    cnt_debug++;
    static int beforEncoder = 0;

    // field check
    if (vfield_count2 != vfield_count2_buff)
    {
        vfield_count2_buff = vfield_count2;
        counter = 0;
    }

    // Top field / bottom field
    switch (counter++)
    {
    case 0:
        ImageCopy(write_buff_addr, PIXEL_HW, PIXEL_VW, ImageData_A, vfield_count2); //  0 - 59行を変換
        break;
    case 1:
        ImageCopy(write_buff_addr, PIXEL_HW, PIXEL_VW, ImageData_A, vfield_count2); // 60 - 119行を変換
        break;
    case 2:
        Extraction_Brightness(ImageData_A, PIXEL_HW, PIXEL_VW, ImageData_B, vfield_count2);
        break;
    case 3:
        Extraction_Brightness(ImageData_A, PIXEL_HW, PIXEL_VW, ImageData_B, vfield_count2);

        sensor_bin = shikiichi_henkan(60, 180, 8); //  60行目 しきい値180 隣同士の差分が8以下なら0x00にする
        // スタートバー検出 bit7とbit0のみ見る
        bar = (shikiichi_henkan(60, 120, 8) |
               shikiichi_henkan(70, 120, 8) |
               shikiichi_henkan(80, 120, 8) |
               shikiichi_henkan(90, 120, 8) |
               shikiichi_henkan(100, 120, 8)) &
              0x3c;
        break;

    case 5:
        // 画像更新ごとの更新処理
        encoder.update();
        encoderAcceleration = encoder.getCnt() - beforEncoder;
        beforEncoder = encoder.getCnt();
        createDeviation();
        createMotorVal();

        // if (pattern != 11)
        // {
        //     createLineFlag(encoder.getCnt() + 12); // ラインを検出する行数)
        //
        // else
        // {
        if (pattern >= 10 && pattern != 101)
        {
            if (encoder.getCnt() <= 15)
            {
                if (cnt1 >= 500)
                {
                    pattern = 101;
                    cnt1 = 0;
                }
            }
            else
            {
                cnt1 = 0;
            }
        }

        flagLineNomal = 57;

        // flagLine = 85 - (encoder.getCnt() * 1);
        flagLine = /*map(encoder.getCnt(), 40, 50, 40, 35)*/40;
        // flagLine = 79 - encoder.getCnt();
        // flagLine = 50;
        if (/*encoder.getCnt() < 47 && */ (pattern == 11 || pattern == 22 || pattern == 52 || pattern == 62))
        {
            flagLine = flagLineNomal;
        }
        else if (flagLine < 36)
        {
            flagLine = 36;
        }
        else if (flagLine > 53)
        {
            flagLine = 53;
        }
        // flagLine = 30;
        // if(encoder.getCnt()<=42){
        // flagLine = 40;
        // }

        lineHeight = /*abs(encoderAcceleration * 3)*/3;
        if (lineHeight > 5)
        {
            lineHeight = 5;
        }
        if (lineHeight < 3)
        {
            lineHeight = 3;
        }
        if (pattern == 0 || pattern == 1)
        {
            flagLine = 90;
            lineHeight = 15;
        }
        else if (pattern == 2)
        {
            flagLine = 60;
            lineHeight = 40;
        }
        createLineFlag(flagLine, lineHeight); // ラインを検出する行数)
                                              // }
        createHandleVal();

        break;

    case 6:
        MTU2TCNT_0 = 0;
        break;

    case 10:

        if (endflag == false)
        {
            // ログ（RAM）記録
            log_data[log_no].cnt_msdwritetime = cnt_msdwritetime;
            log_data[log_no].pattern = pattern;
            log_data[log_no].convertBCD = handleVal;
            log_data[log_no].handle = msd_handle;
            log_data[log_no].hennsa = allDeviation[flagLine]; // 偏差を検出するプログラムを作ったら追加
            log_data[log_no].encoder = encoder.getCnt();
            log_data[log_no].motorL = flagLine /*getImage(-allDeviation[60] + IMAGE_CENTER - 35, 60)*/;
            log_data[log_no].motorR = msd_r;

            log_data[log_no].flagL = lineflag_left;
            log_data[log_no].flagK = lineflag_cross;
            log_data[log_no].flagR = lineflag_right;
            log_data[log_no].flagS = lineflag_center; // センターライン検出フラグを作ったら追加

            log_data[log_no].total = encoder._total_cnt;

            // log_data[log_no].total = encoder.getTotalCount();
            for (int i = 0; i < 160; i++)
            {
                log_data[log_no].imageData0[i] = getImage(i, flagLine);
            }

            log_no++;
        }
        break;

    default:
        break;
    }

    // LED(rgb) on the GR-peach board
    led_m_process(); // LEDの点滅処理を行う

    if (debug_mode != 0 || initFlag != 0)
    {
        return; // デバッグモードなら、ここで割り込み終了
                // 走行プログラムは実行しない
    }

    // モータドライブ基板のLED2個は、スタートバーの反応チェック用 スタート時にLEDが点灯するようにセットする
    led_out(((bar & 0x80) >> 6) | (bar & 0x01));

    // GR-PEACHのUSER_LED(赤色)は、sensor_inp関数の中心2個のどれかが"1"なら、点灯する
    USER_LED = sensor_inp(0x18) != 0x00 ? 1 : 0;

    if (pattern >= 11 && pattern <= 100 && user_button_get() == 1 &&
        log_mode == 2)
    {
        pattern = 101;
        log_mode = 3; // ログ保存終了
    }

    switch (pattern)
    {

    case 0:
        // スイッチ入力待ち

        if (lineflag_cross == 0)
        {
            led_m(10, 1, 0, 0); // スタートバーセットOK状態→赤色点灯

            if (pushsw_get() == 1)
            {
                pattern = 1;
                log_mode = 1; // ログファイルオープン
                cnt1 = 0;
                encoder.clear();
                break;
            }
        }
        else
        {
            led_m(50, 0, 1, 0); // スタートバーセットNO状態→緑色点灯

            if (pushsw_get() == 1)
            {
                pattern = 4;
                log_mode = 1; // ログファイルオープン
                cnt1 = 0;
                break;
            }
            // led_m(50, 0, 1, 0); // スタートバーセットOK状態→緑色点灯
        }
        break;

    case 1:

        // 2秒たったらバーに向けてすすむ
        if (cnt1 >= 500)
        {
            // createBrakeMotorVal(10);
            motor(18, 18);
        }

        handle(allDeviation[95] * 0.5);

        // スタートバーに到着したら
        if (lineflag_cross == 1)
        {
            led_m(50, 0, 0, 1);
            cnt_msdwritetime = 0;
            pattern = 2;
            cnt1 = 0;
            encoder.clear();
            break;
        }
        led_m(10, 1, 1, 0);
        break;

    case 2:
        if (cnt1 <= 100)
        {
            handle(allDeviation[95] * 0.5);
        }
        else
        {
            handle(0);
        }
        handle(0);

        motor(0, 0);

        if (!lineflag_cross && cnt1 >= 1000)
        {
            led_m(50, 0, 1, 0);

            pattern = 3;
        }

        break;

    case 2000:
        handle(0);
        // handle(allDeviation[95]*0.5);

        motor(0, 0);

        if (!lineflag_cross && !lineflag_left && !lineflag_right)
        {
            led_m(50, 0, 1, 0);

            pattern = 2000;
        }
        break;

    case 3:
        handle(handleVal);
        motor(100, 100);
        if (encoder.getTotalCount() >= 100)
        {
            pattern = 4;
            cnt1 = 0;
        }
        break;

    case 4:
        if (lineflag_cross == 0)
        {
            pattern = 10;
            log_mode = 2; // ログ記録中
        }
        break;

    case 10:
        // スタートバーが開いたかチェック
        if (lineflag_cross == 0)
        {
            // スタート！！
            led_m(0, 0, 0, 0);
            cnt_msdwritetime = 0;
            pattern = 11;
            encoder.setvalue(500);
            cnt1 = 0;
            break;
        }
        led_m(10, 1, 1, 0);
        break;

    case 11:
        // 通常トレース
        led_m(50, 1, 1, 1);
        trace_offset = 0;
        if (abs(allDeviation[flagLine - 2]) < 20 /*&& abs(allDeviation[60]) < 15 && abs(allDeviation[90]) < 15 */ && encoder.getTotalCount() >= 700)
        {
            lineSkipDistance = 100;

            laneAfterDistance = 0;
            laneCounterDistance = 320; // 300
            crankDistance = 440;

            // if (encoder.getCnt() <= 30)
            // {

            //     constCrankHandleVal = 25;

            //     constCrankMotorPowerOUT = 90;
            //     crankMotorPowerOUTGain = 0;

            //     constCrankMotorPowerIN = 90;
            //     crankMotorPowerINGain = 0;
            // }
            // else
            // {

            constCrankHandleVal = 42;
            // if(constCrankHandleVal>=49){
            //     constCrankHandleVal=49;
            // }
            //  if(constCrankHandleVal<=35){
            //     constCrankHandleVal=35;
            // }

            crankHandleValGain = 0;

            constCrankMotorPowerOUT = 100;
            crankMotorPowerOUTGain = 0;

            constCrankMotorPowerIN = 100; // 70
            crankMotorPowerINGain = 0;
            // }

            laneStraightMotorPower = 100;

            // GOD
            // laneDistance = 320;
            // LANE_DISTANCE

            // LANE_HANDLE_CALK

            laneHandleVal = 6;
            lanePowerGain = 0.6;

            laneMotorPowerIN = 60;
            laneMotorPowerOUT = 60;

            laneCounterHandleVal = 37;
            laneCounterMotorPowerOUT = 60;
            laneCounterMotorPowerIN = 60;
            //

            lowSpeedLimit = 33;

            lowSpeedCrankHandle = 26;
            lowSpeedCrankDistance = 400;

            lowSpeedLaneHandle = 14;
            lowSpeedLaneDistance = 360;

            if (lineflag_cross)
            {
                pattern = 21;
            }
            else if (lineflag_right)
            {
                pattern = 51;
                trace_offset = 0;
            }
            else if (lineflag_left)
            {
                pattern = 61;
                trace_offset = -0;
            }
        }

        if (encoder._total_cnt >= 1000)
        {
            shioCheck = 100;
        }

        motor(leftMotor, rightMotor);
        handle(handleVal);

        if (encoder.getCourseCount() >= ONEMETOR_PULSE * 58)
        {
            pattern = 101;
            cnt1 = 0;
        }

        break;

    case 21:
        // クロスライン検出時の処理
        led_m(100, 1, 0, 0);
        pattern = 22;
        encoder.clear();
        break;

    case 22:
        // クロスラインを読み飛ばす
        createBrakeMotorVal(43);
        motor(leftBrakeMotor, rightBrakeMotor);
        // motor(leftMotor, rightMotor);

        handle(handleVal);

        if (encoder.getTotalCount() >= lineSkipDistance && !lineflag_left && !lineflag_right && !lineflag_cross)
        {
            encoder.clear();

            pattern = 23;
            cnt1 = 0;
        }
        break;

    case 23:
        // クロスライン後のトレース、クランク検出

        crankHandleVal = constCrankHandleVal + encoder.getCnt() * crankHandleValGain;

        crankMotorPowerOUT = constCrankMotorPowerOUT + encoder.getCnt() * crankMotorPowerOUTGain;
        crankMotorPowerIN = constCrankMotorPowerIN + encoder.getCnt() * crankMotorPowerINGain;
        if (crankMotorPowerIN < -95)
        {
            crankMotorPowerIN = -95;
        }
        if (crankHandleVal > 49)
        {
            crankHandleVal = 49;
        }

        if (encoder.getCnt() <= lowSpeedLimit)
        {
            crankHandleVal = lowSpeedCrankHandle;
            crankDistance = lowSpeedCrankDistance;
        }

        if (encoder.getCnt() >= 45)
        {
            crankMotorPowerOUT = 90;
            crankMotorPowerIN = 90;
        }

        if (lineflag_left)
        {

            // 左クランクと判断]
            encoder.clear();

            led_m(100, 0, 1, 0);
            handle(crankHandleVal);
            motor(crankMotorPowerIN, crankMotorPowerOUT);
            pattern = 30;
            break;
        }
        if (lineflag_right)
        {

            // 右クランクと判断
            encoder.clear();

            led_m(100, 0, 0, 1);
            handle(-crankHandleVal);
            motor(crankMotorPowerOUT, crankMotorPowerIN);
            pattern = 40;

            break;
        }

        // if (encoder._total_cnt >= 1000)
        // {
        //     pattern = 11;
        //     led_m(100, 1, 1, 0);

        //     //encoder.clear();

        //     break;
        // }
        createBrakeMotorVal(43);
        motor(leftBrakeMotor, rightBrakeMotor);
        // motor(leftMotor, rightMotor);

        handle(handleVal);

        break;

    case 30:
        encoder.clear();
        encoder._total_cnt = 0;
        shioCheck = 300;

        pattern = 31;

        break;
    case 40:
        shioCheck = 400;

        encoder.clear();
        encoder._total_cnt = 0;
        pattern = 41;

        break;

    case 31:
        shioCheck = 310;

        handle(crankHandleVal);
        motor(crankMotorPowerIN, crankMotorPowerOUT);

        led_m(100, 0, 1, 0);
        if (encoder._total_cnt >= crankDistance)
        {
            resetFlag = true;
            encoder.clear();
            pattern = 11;
        }
        break;
    case 41:
        shioCheck = 410;

        handle(-crankHandleVal);

        motor(crankMotorPowerOUT, crankMotorPowerIN);
        led_m(100, 0, 1, 0);
        if (encoder._total_cnt >= crankDistance)
        {
            encoder.clear();
            resetFlag = true;
            pattern = 11;
        }
        break;

    case 51:
        // ハーフライン検出時の処理
        led_m(100, 0, 1, 0);
        pattern = 52;

        encoder.clear();
        break;

    case 52:
        // ハーフラインを読み飛ばす
        createBrakeMotorVal(43);
        motor(leftBrakeMotor, rightBrakeMotor);
        // motor(leftMotor, rightMotor);

        handle(handleVal);
        if (encoder.getTotalCount() >= lineSkipDistance && !lineflag_left && !lineflag_right)
        {
            pattern = 53;
            encoder.clear();
        }
        if (lineflag_left || lineflag_cross)
        {
            pattern = 22;
            break;
        }
        if (lineflag_cross == 1)
        {
            pattern = 22;
            break;
        }
        break;

    case 53:
        // ハーフライン後のトレース、レーンチェンジ

        if (!lineflag_center)
        {

            pattern = 54;
            led_m(100, 1, 1, 0);

            encoder.clear();
            break;
        }

        if (encoder._total_cnt >= 1000)
        {
            pattern = 11;
            led_m(100, 1, 1, 0);

            // encoder.clear();
            break;
        }
        createBrakeMotorVal(43);
        motor(leftBrakeMotor, rightBrakeMotor);
        // motor(leftMotor, rightMotor);
        handle(handleVal);
        break;

    case 54:

        // レーンチェンジ終了のチェック
        handle(0);

        createBrakeMotorVal(43);
        motor(leftBrakeMotor, rightBrakeMotor);
        // motor(laneStraightMotorPower, laneStraightMotorPower);
        if (encoder.getTotalCount() >= 0)
        {

            pattern = 55;
            led_m(100, 1, 0, 0);
            LANE_DISTANCE
            // if (encoder.getCnt() <= lowSpeedLimit)
            // {
            //     laneDistance = lowSpeedLaneDistance;
            // }
            encoder.clear();
        }
        break;

    case 55:

        handle(-(LANE_HANDLE_CALK));
        //  createBrakeMotorVal(39);
        // motor(leftBrakeMotor, rightBrakeMotor);
        motor(laneMotorPowerOUT, laneMotorPowerIN);
        if (encoder.getTotalCount() >= laneDistance)
        {

            pattern = 56;

            led_m(100, 0, 0, 0);
            encoder.clear();
        }
        break;

    case 56:

        handle(0);
        motor(laneStraightMotorPower, laneStraightMotorPower);
        // createBrakeMotorVal(39);
        // motor(leftBrakeMotor, rightBrakeMotor);
        if (encoder.getTotalCount() >= laneAfterDistance)
        {

            pattern = 57;

            led_m(100, 0, 0, 1);
            LANE_COUNTER_DISTANCE
            encoder.clear();
        }
        break;

    case 57:

        handle(LANE_COUNTER_HANDLE_CALK);
        //  createBrakeMotorVal(39);
        // motor(leftBrakeMotor, rightBrakeMotor);
        motor(laneCounterMotorPowerIN, laneCounterMotorPowerOUT);
        if (encoder.getTotalCount() >= laneCounterDistance)
        {

            pattern = 11;
            led_m(100, 0, 0, 1);
            encoder.clear();
            resetFlag = true;
        }
        break;

    case 61:
        // ハーフライン検出時の処理
        led_m(100, 0, 1, 0);
        pattern = 62;

        encoder.clear();
        break;

    case 62:
        // ハーフラインを読み飛ばす
        createBrakeMotorVal(43);
        motor(leftBrakeMotor, rightBrakeMotor);
        // motor(leftMotor, rightMotor);

        handle(handleVal);
        if (encoder.getTotalCount() >= lineSkipDistance && !lineflag_left && !lineflag_right)
        {
            pattern = 63;
            encoder.clear();
        }
        if ((lineflag_right || lineflag_cross))
        {
            pattern = 22;
            break;
        }
        if (lineflag_cross == 1)
        {
            pattern = 22;
            break;
        }
        break;

    case 63:
        // ハーフライン後のトレース、レーンチェンジ

        if (!lineflag_center)
        {

            pattern = 64;
            led_m(100, 1, 1, 0);

            encoder.clear();
            break;
        }

        if (encoder._total_cnt >= 1000)
        {
            pattern = 11;
            led_m(100, 1, 1, 0);

            // encoder.clear();
            break;
        }
        createBrakeMotorVal(43);
        motor(leftBrakeMotor, rightBrakeMotor);
        // motor(leftMotor, rightMotor);
        handle(handleVal);
        break;

    case 64:

        // レーンチェンジ終了のチェック
        handle(0);
        // createBrakeMotorVal(39);
        // motor(leftBrakeMotor, rightBrakeMotor);
        motor(laneStraightMotorPower, laneStraightMotorPower);
        if (encoder.getTotalCount() >= 0)
        {

            pattern = 65;
            led_m(100, 1, 0, 0);
            LANE_DISTANCE
            // if (encoder.getCnt() <= lowSpeedLimit)
            // {
            //     laneDistance = lowSpeedLaneDistance;
            // }
            encoder.clear();
        }
        break;

    case 65:

        handle(LANE_HANDLE_CALK);
        //  createBrakeMotorVal(39);
        // motor(leftBrakeMotor, rightBrakeMotor);
        motor(laneMotorPowerIN, laneMotorPowerOUT);
        if (encoder.getTotalCount() >= laneDistance)
        {

            pattern = 66;
            led_m(100, 0, 0, 0);

            encoder.clear();
        }
        break;

    case 66:

        handle(0);
        //  createBrakeMotorVal(39);
        // motor(leftBrakeMotor, rightBrakeMotor);
        motor(laneStraightMotorPower, laneStraightMotorPower);
        if (encoder.getTotalCount() >= laneAfterDistance)
        {

            pattern = 67;

            led_m(100, 0, 0, 1);
            LANE_COUNTER_DISTANCE
            encoder.clear();
        }
        break;

    case 67:

        handle(-(LANE_COUNTER_HANDLE_CALK));
        // createBrakeMotorVal(39);
        // motor(leftBrakeMotor, rightBrakeMotor);
        motor(laneCounterMotorPowerOUT, laneCounterMotorPowerIN);
        if (encoder.getTotalCount() >= laneCounterDistance)
        {

            pattern = 11;
            led_m(100, 0, 0, 1);
            encoder.clear();
            resetFlag = true;
        }
        break;

    case 101:
        // 終了　ログ保存中など
        led_m(100, 0, 1, 1);
        static int hanva;
        if (encoder.getCnt() != 1 && encoderAcceleration == 0)
        {
            handle(handleVal);
            hanva = handleVal;
        }
        else
        {
            handle(hanva);
        }
        endflag = true;
        if (pushsw_get())
        {
            motor(100, 100);
        }
        else
        {
            motor(-encoder.getCnt() / 2, -encoder.getCnt() / 2);
        }
        break;

    default:
        // どれでもない場合は待機状態に戻す
        pattern = 0;
        break;
    }
}

////****************************************************************
// functions ( on GR-PEACH board )
////****************************************************************
//------------------------------------------------------------------//
// user_button_get Function
//------------------------------------------------------------------//
unsigned char user_button_get(void)
{
    return (~user_botton) & 0x1; // Read ports with switches
}

//------------------------------------------------------------------//
// led_m Function
//------------------------------------------------------------------//
void led_m(int time, int r, int g, int b)
{
    mled_time = time;
    mled_r = r;
    mled_g = g;
    mled_b = b;
}

//------------------------------------------------------------------//
// led_m_process Function for only interrupt
//------------------------------------------------------------------//
void led_m_process(void)
{
    static int cnt_led_m = 0;

    if (cnt_led_m < mled_time * 5)
    {
        LED_R = mled_r;
        LED_G = mled_g;
        LED_B = mled_b;
    }
    else
    {
        LED_R = 0;
        LED_G = 0;
        LED_B = 0;
    }

    cnt_led_m++;
    if (cnt_led_m >= 500)
    {
        cnt_led_m = 0;
    }
}

///*****************************************************************
// functions ( on Motor drive board )
///*****************************************************************
//------------------------------------------------------------------//
// led_out Function
//------------------------------------------------------------------//
void led_out(int led)
{
    led = ~led;
    LED_3 = led & 0x1;
    LED_2 = (led >> 1) & 0x1;
}

//------------------------------------------------------------------//
// pushsw_get Function
//------------------------------------------------------------------//
unsigned char pushsw_get(void)
{
    return (!push_sw); // Read ports with switches
}

//------------------------------------------------------------------//
// motor speed control(PWM)
// Arguments: motor:-100 to 100
// Here, 0 is stop, 100 is forward, -100 is reverse
//------------------------------------------------------------------//
void motor(int accele_l, int accele_r)
{
    int sw_data;

    if (accele_l > MAX_MOTOR_POWER)
    {
        accele_l = MAX_MOTOR_POWER;
    }
    if (accele_l < -MAX_MOTOR_POWER)
    {
        accele_l = -MAX_MOTOR_POWER;
    }

    if (accele_r > MAX_MOTOR_POWER)
    {
        accele_r = MAX_MOTOR_POWER;
    }
    if (accele_r < -MAX_MOTOR_POWER)
    {
        accele_r = -MAX_MOTOR_POWER;
    }

    msd_l = accele_l;
    msd_r = accele_r;

    // Left Motor Control
    if (accele_l >= 0)
    {
        // forward
        Left_motor_signal = 1;
        MTU2TGRC_4 = (long)(MOTOR_PWM_CYCLE - 1) * accele_l / 100;
    }
    else
    {
        // reverse
        Left_motor_signal = 0;
        MTU2TGRC_4 = (long)(MOTOR_PWM_CYCLE - 1) * (-accele_l) / 100;
    }

    // Right Motor Control
    if (accele_r >= 0)
    {
        // forward
        Right_motor_signal = 1;
        MTU2TGRD_4 = (long)(MOTOR_PWM_CYCLE - 1) * accele_r / 100;
    }
    else
    {
        // reverse
        Right_motor_signal = 0;
        MTU2TGRD_4 = (long)(MOTOR_PWM_CYCLE - 1) * (-accele_r) / 100;
    }
}

//------------------------------------------------------------------//
// handle Function
//------------------------------------------------------------------//
void handle(int angle)
{
    msd_handle = -angle;
    // When the servo move from left to right in reverse, replace "-" with "+"
    MTU2TGRD_0 = SERVO_CENTER - angle * HANDLE_STEP;
}

///*****************************************************************
// functions ( on Shield board )
///*****************************************************************
//------------------------------------------------------------------//
// Dipsw get Function
//------------------------------------------------------------------//
unsigned char dipsw_get(void)
{
    return (dipsw.read() & 0x0f);
}

//------------------------------------------------------------------//
// sensor Function
//------------------------------------------------------------------//
unsigned char sensor_inp(unsigned char mask)
{
    return (sensor_bin & mask); // 中
}

///*********************************************************************
// モジュール名 getCompileYear
// 処理概要     コンパイルした時の年を取得
// 引数　       __DATE__ のポインタ
// 戻り値       年
///*********************************************************************
int getCompileYear(const char *p)
{
    int i;

    i = atoi(p + 7);
    if (i < 1980 || i > 2107)
        i = 2019;

    return i;
}

///*********************************************************************
// モジュール名 getCompileMonth
// 処理概要     コンパイルした時の月を取得
// 引数　       __DATE__ のポインタ
// 戻り値       月
///*********************************************************************
int getCompileMonth(const char *p)
{
    int i, r;

    for (i = 0; i < 12; i++)
    {
        r = strncmp(monthStr + i * 3, p, 3);
        if (r == 0)
            return i + 1;
    }
    return 1;
}

///*********************************************************************
// モジュール名 getCompileDay
// 処理概要     コンパイルした時の日を取得
// 引数　       __DATE__ のポインタ
// 戻り値       日
///*********************************************************************
int getCompileDay(const char *p)
{
    int i;

    i = atoi(p + 4);
    if (i < 1 || i > 31)
        i = 1;

    return i;
}

///*********************************************************************
// モジュール名 getCompileHour
// 処理概要     コンパイルした時の時を取得
// 引数　       __TIME__ のポインタ
// 戻り値       時
///*********************************************************************
int getCompileHour(const char *p)
{
    int i;

    i = atoi(p);
    if (i < 0 || i > 23)
        i = 0;

    return i;
}

///*********************************************************************
// モジュール名 getCompilerMinute
// 処理概要     コンパイルした時の分を取得
// 引数　       __TIME__ のポインタ
// 戻り値       分
///*********************************************************************
int getCompilerMinute(const char *p)
{
    int i;

    i = atoi(p + 3);
    if (i < 0 || i > 59)
        i = 0;

    return i;
}

///*********************************************************************
// モジュール名 getCompilerSecond
// 処理概要     コンパイルした時の秒を取得
// 引数　       __TIME__ のポインタ
// 戻り値       秒
///*********************************************************************
int getCompilerSecond(const char *p)
{
    int i;

    i = atoi(p + 6);
    if (i < 0 || i > 59)
        i = 0;

    return i;
}

///*********************************************************************
// char型データの値をlong型変数に2進数で変換
// 引数　 unsigned char 変換元の8bitデータ
// 戻り値 unsigned long 変換先の変数(0～11111111) ※0か1しかありません
///*********************************************************************
unsigned long convertBCD_CharToLong(unsigned char hex)
{
    int i;
    unsigned long l = 0;

    for (i = 0; i < 8; i++)
    {
        l *= 10;
        if (hex & 0x80)
            l += 1;
        hex <<= 1;
    }

    return l;
}

///*********************************************************************
// クロスライン検出処理
// 戻り値 0:クロスラインなし 1:あり
///*********************************************************************
int check_crossline(void)
{
    unsigned char b;
    int ret;

    ret = 0;
    b = sensor_inp(MASK3_3);
    if (b == 0xe7)
    {
        ret = 1;
    }
    return ret;
}

///*********************************************************************
// 右ハーフライン検出処理
// 戻り値 0:なし 1:あり
///*********************************************************************
int check_rightline(void)
{
    unsigned char b;
    int ret;

    ret = 0;
    b = sensor_inp(MASK4_4);
    if (b == 0x0f || b == 0x1f || b == 0x3f || b == 0x7f)
    {
        ret = 1;
    }
    return ret;
}

///*********************************************************************
// 左ハーフライン検出処理
// 戻り値 0:なし 1:あり
///*********************************************************************
int check_leftline(void)
{
    unsigned char b;
    int ret;

    ret = 0;
    b = sensor_inp(MASK4_4);
    if (b == 0xf0 || b == 0xf8 || b == 0xfc || b == 0xfe)
    {
        ret = 1;
    }
    return ret;
}

///********************************************************************
// 指定した行数の８点を取得し、しきい値を自動で調整し2進数8ビットに変換する
// 引数：行数(0-119), しきい値, 差
// 戻り値：センサの８ビット
///********************************************************************
unsigned char shikiichi_henkan(int gyou, int s, int sa)
{
    int max, min, i, shiki;
    int d[8];
    int sa_7_6, sa_6_5, sa_5_4, sa_4_3, sa_3_2, sa_2_1, sa_1_0;
    unsigned char ret;

    d[7] = getImage(31, gyou);
    d[6] = getImage(43, gyou);
    d[5] = getImage(54, gyou);
    d[4] = getImage(71, gyou);
    d[3] = getImage(88, gyou);
    d[2] = getImage(105, gyou);
    d[1] = getImage(116, gyou);
    d[0] = getImage(128, gyou);

    min = max = d[0];
    for (i = 1; i < 8; i++)
    {
        if (max <= d[i])
        {
            max = d[i]; // 8個のうち、最大を見つける
        }
        if (min >= d[i])
        {
            min = d[i]; // 8個のうち、最小を見つける
        }
    }

    // 隣同士の差の絶対値
    sa_7_6 = abs(d[7] - d[6]);
    sa_6_5 = abs(d[6] - d[5]);
    sa_5_4 = abs(d[5] - d[4]);
    sa_4_3 = abs(d[4] - d[3]);
    sa_3_2 = abs(d[3] - d[2]);
    sa_2_1 = abs(d[2] - d[1]);
    sa_1_0 = abs(d[1] - d[0]);

    if (max >= s)
    {
        // 最大値がs以上なら、sをしきい値とする
        shiki = s;
    }
    else if (sa_7_6 >= sa || sa_6_5 >= sa || sa_5_4 >= sa ||
             sa_4_3 >= sa || sa_3_2 >= sa || sa_2_1 >= sa || sa_1_0 >= sa)
    {
        // 隣同士の差が１つでも「差」以上なら、８点の（最大値－最小値）×0.7 + 最小値　をしきい値とする
        shiki = (max - min) * 7 / 10 + min;
    }
    else
    {
        // 当てはまらなければ、しきい値を256とする、すなわちすべて0となる
        shiki = 256;
    }

    // d[7]～d[0]をbit7～bit0に割り当てる
    ret = 0;
    for (i = 7; i >= 0; i--)
    {
        ret <<= 1;
        ret |= (d[i] >= shiki ? 1 : 0);
    }

    return ret;
}

///********************************************************************
// イメージ領域のx,yの値(0-255)を取得
// 引数：x列数(0-159) , y行数(0-119)
// 戻り値：0～255
///********************************************************************
char getImage(int ix, int iy)
{
    return ImageData_B[ix + 160U * iy];
}
///********************************************************************
// ラインフラグを生成する関数
// 引数なし(自分でプログラムを変えてフラグを生成するために必要な行数を指定できるようにして　　初期６０行目)
// 戻り値：0～255
///********************************************************************
void createLineFlag(int rowNum, int height)
{
    volatile int crosslineWidth = rowNum + 20; // クロスラインの検出に中心から何列のデータを使うか指定(コースの幅より外側のデータを使わないため)
    if (pattern == 11)
    {
        crosslineWidth = 70;
    }

    if (pattern == 1 || pattern == 2)
    {
        crosslineWidth = 60;
        height = 10;
    }
    volatile int centerRowNum = 50; /* map(encoder.getCnt(), 30, 30, 54, 54); // ラインを検出する行数*/

    volatile int centerWidth = centerRowNum - 2; // 中心線があるかの検出に中心から何列のデータを使うか指定(中心線の幅数)

    volatile int brightnessThreshold = 0; // 明るさの閾値明るさは255段階になっていて閾値より下の値が来ていた場合は切り捨てる
    volatile int maxBrightness = 0;
    volatile int bigImageData[IMAGE_WIDTH][IMAGE_HEIGHT]; // 特定の一行のみの画像の明るさデータが格納される配列

    volatile int imageData[IMAGE_WIDTH];  // 特定の一行のみの画像の明るさデータが格納される配列
    volatile int centerData[IMAGE_WIDTH]; // 特定の一行のみの画像の明るさデータが格納される配列

    volatile int leftCount = 0;   // 画像の左側に閾値以上の値がどれくらいあるかをカウントする
    volatile int rightCount = 0;  // 画像の右側に閾値以上の値がどれくらいあるかをカウントする
    volatile int centerCount = 0; // 画像のセンターライン付近に閾値以上の値がどれくらいあるかをカウントする

    volatile int crossCountThreshold = crosslineWidth - 1; // 画像のクロスラインのカウント数の閾値

    volatile int centerCountThreshold = 10; // 画像のセンターラインのカウント数の閾値

    // if (pattern != 11)
    // {
    //     rowNum = 60;
    // }

    // imageDataに画像データを格納する
    for (int y = rowNum; y < rowNum + height; y++)
    {
        for (int x = 0; x < IMAGE_WIDTH; x++)
        {
            bigImageData[x][y] = getImage(x, y);
            imageData[x] = 0;
            if (maxBrightness < bigImageData[x][y])
            {
                maxBrightness = bigImageData[x][y];
            }
            // imageData[x] = getImage(x, rowNum);
        }
    }
    brightnessThreshold = maxBrightness * 0.69;
    // brightnessThreshold=230;
    if (pattern == 1 || pattern == 2)
    {
        // brightnessThreshold = maxBrightness * 0.4;
        brightnessThreshold = 90;
        centerWidth = 100;
        crossCountThreshold = 59;
    }
    else if (pattern != 11)
    {
        crossCountThreshold = crosslineWidth - 10;
        brightnessThreshold = maxBrightness * 0.68;
    }
    for (int y = rowNum; y < rowNum + height; y++)
    {
        for (int x = 0; x < IMAGE_WIDTH; x++)
        {
            if (bigImageData[x][y] > imageData[x])
            {
                imageData[x] = bigImageData[x][y];
            }
        }
    }
    for (int x = 0; x < IMAGE_WIDTH; x++)
    {
        centerData[x] = getImage(x, centerRowNum);
    }

    // レフトライン検出処理
    for (int x = IMAGE_CENTER - crosslineWidth / 2; x < IMAGE_CENTER; x++) // 画像の中心-クロスラインの幅の半分(レフトラインの幅)から画像の中心まで
    {
        if (imageData[x] > brightnessThreshold) // その画素が閾値よりも大きい値が来ていたらカウント++
        {
            leftCount++;
        }
    }

    // leftCountがcrossCountThresholdの半分(レフトラインの判定基準)より多く判定されたらフラグtrue
    if (leftCount > crossCountThreshold / 2)
    {
        lineflag_left = true;
    }
    else
    {
        lineflag_left = false;
    }

    // ここからは自分で読んで理解して！

    //  ライトライン検出処理
    for (int x = IMAGE_CENTER; x < IMAGE_CENTER + crosslineWidth / 2; x++)
    {
        if (imageData[x] > brightnessThreshold)
        {
            rightCount++;
        }
    }

    if (rightCount > crossCountThreshold / 2)
    {
        lineflag_right = true;
    }
    else
    {
        lineflag_right = false;
    }

    // クロスライン判定(ライトラインとレフトラインが検出されてたらクロスライン判定)
    if (lineflag_right == true && lineflag_left == true)
    {
        lineflag_cross = true;
    }
    else
    {
        lineflag_cross = false;
    }

    // 中心線があるかの検出
    for (int x = IMAGE_WIDTH / 2 - centerWidth / 2; x < IMAGE_WIDTH / 2 + centerWidth / 2; x++)
    {
        if (centerData[x] > 170)
        {
            centerCount++;
        }
    }

    if (centerCount > centerCountThreshold)
    {
        lineflag_center = true;
    }
    else
    {
        lineflag_center = false;
    }
}
// 偏差を作る関数
void createDeviation(void)
{
    volatile float brightnessThreshold = 0.87;  // 明るさの閾値倍率
    volatile int minasDifferenceThreshold = -8; // 左側差分検出の閾値
    volatile int plusDifferenceThreshold = 8;   // 右側差分検出の閾値

    volatile int differenceThresholdY = 5; // 一行下との検出された場所による外れ値検出の閾値
    if (pattern != 11 || debug_mode == 3)
    {
        differenceThresholdY = 10000;
        minasDifferenceThreshold = -17;
        plusDifferenceThreshold = 17;
    }

    volatile signed int allImageData[IMAGE_HEIGHT][IMAGE_WIDTH]; // 画像データが格納された配列
    volatile signed int maxBrightness = 0;                       // 明るさの最大値

    volatile signed int leftExceedingXPositions[IMAGE_HEIGHT][IMAGE_WIDTH];  // 左側の差分が検出された場所(添え字二個目は検出されたのの何個目かを表す)
    volatile signed int leftExceedingXPositionsCount[IMAGE_HEIGHT];          // 左側の差分が検出された個数
    volatile signed int rightExceedingXPositions[IMAGE_HEIGHT][IMAGE_WIDTH]; // 右側の差分が検出された場所(添え字二個目は検出されたのの何個目かを表す)
    volatile signed int rightExceedingXPositionsCount[IMAGE_HEIGHT];         // 右側の差分が検出された個数

    volatile signed int leftYDifference[IMAGE_HEIGHT][IMAGE_WIDTH];  // 左側の複数検出された差分の中心との距離
    volatile signed int rightYDifference[IMAGE_HEIGHT][IMAGE_WIDTH]; // 右側の複数検出された差分の中心との距離

    volatile signed int minLeftXDifference[IMAGE_HEIGHT];  // 左側の複数検出された差分の中でどれが一行下の中心と近いかを検出するための変数
    volatile signed int minRightXDifference[IMAGE_HEIGHT]; // 右側の複数検出された差分の中でどれが一行下の中心と近いかを検出するための変数

    volatile signed int rightCenterCount[IMAGE_HEIGHT]; // 右側の差分が検出された場所の何個目がセンターラインかを示す
    volatile signed int leftCenterCount[IMAGE_HEIGHT];  // 左側の差分が検出された場所の何個目がセンターラインかを示す

    volatile signed int minLeft[IMAGE_HEIGHT];  // 右側の差分が検出された場所の何個目がセンターラインかを示す
    volatile signed int minRight[IMAGE_HEIGHT]; // 右側の差分が検出された場所の何個目がセンターラインかを示す

    volatile static signed int beforDeviationThreshold = 15;
    // 変数の初期化
    for (int y = 0; y < IMAGE_HEIGHT; y++)
    {
        leftExceedingXPositionsCount[y] = 0;
        rightExceedingXPositionsCount[y] = 0;
        minLeftXDifference[y] = 160;
        minRightXDifference[y] = 160;
    }

    // getimageから配列に格納
    for (int y = 0; y < IMAGE_HEIGHT; y++)
    {
        for (int x = 0; x < IMAGE_WIDTH; x++)
        {
            allImageData[y][x] = getImage(x, y);
            if (allImageData[y][x] > maxBrightness) // 同時に最大光度も記録
            {
                maxBrightness = allImageData[y][x];
            }
        }
    }

    // 最大光度×閾値以上の値だった場所は255にする(差分を大きくするために二値化に近い処理を行う)
    for (int y = 0; y < IMAGE_HEIGHT; y++)
    {
        for (int x = 0; x < IMAGE_WIDTH; x++)
        {
            if (allImageData[y][x] > maxBrightness * brightnessThreshold)
            {
                allImageData[y][x] = 255;
            }
        }
    }

    // 元データの特定ドットとその右隣のドットの光度の差分を検出する
    for (int y = 0; y < IMAGE_HEIGHT; y++)
    {
        for (int x = 0; x < IMAGE_WIDTH; x++)
        {
            if (x < IMAGE_RIGHT_EDGE)
            {
                difference[y][x] = allImageData[y][x] - allImageData[y][x + 1];
            }
            else
            {
                difference[y][x] = 0;
            }
        }
    }

    // 差分が検出されたところが線の右端か左端かを判断してそれが何列目かを記録する
    for (int y = 0; y < IMAGE_HEIGHT; y++)
    {
        for (int x = 0; x < IMAGE_WIDTH; x++)
        {
            if (difference[y][x] < minasDifferenceThreshold)
            {
                leftExceedingXPositions[y][leftExceedingXPositionsCount[y]] = x;
                leftExceedingXPositionsCount[y]++;
            }
            if (difference[y][x] > plusDifferenceThreshold)
            {
                rightExceedingXPositions[y][rightExceedingXPositionsCount[y]] = x;
                rightExceedingXPositionsCount[y]++;
            }
        }
        if (leftExceedingXPositionsCount[y] == 0) // 検出された場所がなかったら中心付近の値を入れる
        {
            leftExceedingXPositions[y][leftExceedingXPositionsCount[y]] = 70;
            leftExceedingXPositionsCount[y]++;
        }
        if (rightExceedingXPositionsCount[y] == 0)
        {
            rightExceedingXPositions[y][rightExceedingXPositionsCount[y]] = 90;
            rightExceedingXPositionsCount[y]++;
        }
    }

    // 画像の一番下から数行は差分が検出された場所を真ん中辺りにする
    for (int y = IMAGE_BOTTOM_EDGE; y > IMAGE_BOTTOM_EDGE - 1; y--)
    {
        // 画像の一番下の行は検出された場所の一つ目を中心とする
        leftCenterCount[y] = 0;
        rightCenterCount[y] = 0;

        leftExceedingXPositions[y][leftCenterCount[y]] = IMAGE_CENTER - 10;
        rightExceedingXPositions[y][rightCenterCount[y]] = IMAGE_CENTER + 10;
    }
    // int BOTTOMminLeft=100;
    // int BOTTOMminRight=100;

    //  for (int count = 0; count < leftExceedingXPositionsCount[IMAGE_BOTTOM_EDGE]; count++)
    //     {
    //         // 検出されたポイントの個数のcount個目の一列下のセンターラインの検出場所との差＝検出されたポイントの個数のcount個目のX座標-一列下の既に検出された中心の場所
    //         if (abs(leftExceedingXPositions[IMAGE_BOTTOM_EDGE][count]-80) < BOTTOMminLeft)
    //         {
    //             // 差分の最小値を記録
    //             BOTTOMminLeft=leftExceedingXPositions[IMAGE_BOTTOM_EDGE][count]-80;
    //             leftCenterCount[IMAGE_BOTTOM_EDGE] = count;
    //         }

    //     }
    //  for (int count = 0; count < rightExceedingXPositionsCount[IMAGE_BOTTOM_EDGE]; count++)
    //     {
    //      if (abs(rightExceedingXPositions[IMAGE_BOTTOM_EDGE][count]-80) < BOTTOMminRight)
    //         {
    //             // 差分の最小値を記録
    //             BOTTOMminRight=rightExceedingXPositions[IMAGE_BOTTOM_EDGE][count]-80;
    //             rightCenterCount[IMAGE_BOTTOM_EDGE] = count;
    //         }
    //     }

    //     if(BOTTOMminLeft>40){
    //     leftCenterCount[IMAGE_BOTTOM_EDGE] = 0;
    //     leftExceedingXPositions[IMAGE_BOTTOM_EDGE][leftCenterCount[IMAGE_BOTTOM_EDGE]] = IMAGE_CENTER - 10;

    //     }
    //       if(BOTTOMminRight>40){
    //     rightCenterCount[IMAGE_BOTTOM_EDGE] = 0;
    //     rightExceedingXPositions[IMAGE_BOTTOM_EDGE][rightCenterCount[IMAGE_BOTTOM_EDGE]] = IMAGE_CENTER - 10;

    //     }

    // 下から順番に一番下の行の中心線からどの検出された点が一番近いかを検出し、一番近かったものをセンターラインとする
    for (int y = IMAGE_BOTTOM_EDGE - 1; y >= 0; y--)
    {
        minLeft[y] = 255;
        minRight[y] = 255;

        // if (pattern == 11)
        // {
        for (int count = 0; count < leftExceedingXPositionsCount[y]; count++)
        {
            // 検出されたポイントの個数のcount個目の一列下のセンターラインの検出場所との差＝検出されたポイントの個数のcount個目のX座標-一列下の既に検出された中心の場所
            leftYDifference[y][count] = abs(leftExceedingXPositions[y][count] - leftExceedingXPositions[y + 1][leftCenterCount[y + 1]]);
            if (leftYDifference[y][count] < minLeftXDifference[y])
            {
                // 差分の最小値を記録
                minLeftXDifference[y] = leftYDifference[y][count];
                // 検出されたポイントの何個目が中心かを記録
                leftCenterCount[y] = count;
            }
        }
        // }
        // else
        // {

        //     for (int count = 0; count < leftExceedingXPositionsCount[y]; count++)
        //     {
        //         // 検出されたポイントの個数のcount個目の一列下のセンターラインの検出場所との差＝検出されたポイントの個数のcount個目のX座標-一列下の既に検出された中心の場所
        //         // leftYDifference[y][count] = abs(leftExceedingXPositions[y][count] - leftExceedingXPositions[y + 1][leftCenterCount[y + 1]]);
        //         if (abs(leftExceedingXPositions[y][count] - 80) < minLeft[y])
        //         {
        //             minLeft[y] = abs(leftExceedingXPositions[y][count] - 80);
        //             // 差分の最小値を記録
        //             // minLeftXDifference[y] = leftYDifference[y][count];
        //             // 検出されたポイントの何個目が中心かを記録
        //             leftCenterCount[y] = count;
        //         }
        //     }
        // }
        if (pattern == 11 || debug_mode == 3)
        {
            if (abs(leftExceedingXPositions[y][leftCenterCount[y]] - leftExceedingXPositions[y + 1][leftCenterCount[y + 1]]) > differenceThresholdY && y < 100)
            {
                leftCenterCount[y] = leftCenterCount[y + 1];
                leftExceedingXPositions[y][leftCenterCount[y]] = leftExceedingXPositions[y + 1][leftCenterCount[y + 1]];
            }
        }
        else
        {
            // if (abs(leftExceedingXPositions[y][leftCenterCount[y]] - leftExceedingXPositions[y + 1][leftCenterCount[y + 1]]) > differenceThresholdY)
            // {
            //     leftCenterCount[y] = leftCenterCount[y + 1];
            //     leftExceedingXPositions[y][leftCenterCount[y]] = leftExceedingXPositions[y + 1][leftCenterCount[y + 1]];
            // }

            if (abs(leftExceedingXPositions[y][leftCenterCount[y]] - 80) >= 20)
            {
                leftExceedingXPositions[y][leftCenterCount[y]] = 80;
            }
        }
        // if (pattern == 11)
        // {
        for (int count = 0; count < rightExceedingXPositionsCount[y]; count++)
        {
            rightYDifference[y][count] = abs(rightExceedingXPositions[y][count] - rightExceedingXPositions[y + 1][rightCenterCount[y + 1]]);
            if (rightYDifference[y][count] < minRightXDifference[y])
            {
                minRightXDifference[y] = rightYDifference[y][count];
                rightCenterCount[y] = count;
            }
        }
        // }
        // else
        // {
        //     for (int count = 0; count < rightExceedingXPositionsCount[y]; count++)
        //     {

        //         if (abs(rightExceedingXPositions[y][count] - 80) < minRight[y])
        //         {
        //             minRight[y] = abs(rightExceedingXPositions[y][count] - 80);
        //             // minRightXDifference[y] = rightYDifference[y][count];
        //             rightCenterCount[y] = count;
        //         }
        //     }
        // }
        if (pattern == 11 || debug_mode == 3)
        {
            if (abs(rightExceedingXPositions[y][rightCenterCount[y]] - rightExceedingXPositions[y + 1][rightCenterCount[y + 1]]) > differenceThresholdY && y < 100)
            {
                rightCenterCount[y] = rightCenterCount[y + 1];
                rightExceedingXPositions[y][rightCenterCount[y]] = rightExceedingXPositions[y + 1][rightCenterCount[y + 1]];
            }
        }
        else
        {
            // if (abs(rightExceedingXPositions[y][rightCenterCount[y]] - rightExceedingXPositions[y + 1][rightCenterCount[y + 1]]) > differenceThresholdY)
            // {
            //     rightCenterCount[y] = rightCenterCount[y + 1];
            //     rightExceedingXPositions[y][rightCenterCount[y]] = rightExceedingXPositions[y + 1][rightCenterCount[y + 1]];
            // }
            if (abs(rightExceedingXPositions[y][rightCenterCount[y]] - 80) >= 20)
            {
                rightExceedingXPositions[y][rightCenterCount[y]] = 80;
            }
        }
    }

    // 検出された中心線の場所が一列下の中心線との差分が閾値以上だったら外れ値として一列下の値を代入する
    // for (int y = IMAGE_BOTTOM_EDGE - 10; y > 0; y--)
    // {
    //     if (abs(leftExceedingXPositions[y][leftCenterCount[y]] - leftExceedingXPositions[y + 1][leftCenterCount[y + 1]]) > differenceThresholdY)
    //     {
    //         leftExceedingXPositions[y][leftCenterCount[y]] = leftExceedingXPositions[y + 1][leftCenterCount[y + 1]];
    //     }
    //     if (abs(rightExceedingXPositions[y][rightCenterCount[y]] - rightExceedingXPositions[y + 1][rightCenterCount[y + 1]]) > differenceThresholdY)
    //     {
    //         rightExceedingXPositions[y][rightCenterCount[y]] = rightExceedingXPositions[y + 1][rightCenterCount[y + 1]];
    //     }
    // }

    // 最終的な画像の中心と中心線のずれ(偏差)をグローバル変数に代入
    allDeviation[IMAGE_HEIGHT - 1] = 0;
    for (int y = IMAGE_HEIGHT - 2; y > 1; y--)
    {
        leftDeviation[y] = IMAGE_CENTER - leftExceedingXPositions[y][leftCenterCount[y]];
        rightDeviation[y] = IMAGE_CENTER - rightExceedingXPositions[y][rightCenterCount[y]];
        allDeviation[y] = leftDeviation[y] + rightDeviation[y];
        // if (abs(allDeviation[y + 1] - (IMAGE_CENTER - (leftExceedingXPositions[y][leftCenterCount[y]] + rightExceedingXPositions[y][rightCenterCount[y]]) / 2)) < beforDeviationThreshold && y < 110)
        // {
        allDeviation[y] = IMAGE_CENTER - (leftExceedingXPositions[y][leftCenterCount[y]] + rightExceedingXPositions[y][rightCenterCount[y]]) / 2;
        // }
        // else
        // {
        //     allDeviation[y] = allDeviation[y + 1];
        // }
        // if (resetFlag == true)
        // {
        //     resetFlag = false;
        //     allDeviation[y] = IMAGE_CENTER - (leftExceedingXPositions[y][leftCenterCount[y]] + rightExceedingXPositions[y][rightCenterCount[y]]) / 2;
        // }
    }
}

// void createMotorVal(void)
// {
//     volatile signed int accelerationBrakeGain = 12;
//     volatile signed int targetSpeed = 53;
//     volatile signed int neutralThrottle = 60;
//     volatile signed int brakeThrottle = 0;

//     volatile signed int limitAcceleration = 3;
//     if (targetSpeed < encoder.getCnt())
//     {
//         leftMotor = MAX_MOTOR_POWER - encoderAcceleration * accelerationBrakeGain;
//         rightMotor = MAX_MOTOR_POWER - encoderAcceleration * accelerationBrakeGain;
//     }
//     else
//     {
//         leftMotor = MAX_MOTOR_POWER;
//         rightMotor = MAX_MOTOR_POWER;
//     }
// if (encoder.getCnt() >= targetSpeed)
// {
// leftMotor = neutralThrottle;
// rightMotor = neutralThrottle;

// if (encoderAcceleration >= limitAcceleration)
// {
// leftMotor -= brakeThrottle;
// rightMotor -= brakeThrottle;
//}
//}
//}

void createMotorVal(void)
{
    volatile signed int accelerationBrakeGain = 0;
    volatile signed int targetSpeed = 555;
    volatile signed int neutralThrottle = 80;

    volatile signed int brakeThrottle = 0;

    volatile signed int limitAcceleration = 5;
    leftMotor = MAX_MOTOR_POWER - encoderAcceleration * accelerationBrakeGain;
    rightMotor = MAX_MOTOR_POWER - encoderAcceleration * accelerationBrakeGain;

    if (encoder.getCnt() >= targetSpeed)
    {
        leftMotor = neutralThrottle;
        rightMotor = neutralThrottle;

        if (encoderAcceleration >= limitAcceleration)
        {
            leftMotor = brakeThrottle;
            rightMotor = brakeThrottle;
        }
    }
}

void createBrakeMotorVal(int targetSpeed)
{
    volatile signed int largeSpeedThreshold = 5;
    volatile signed int mediumSpeedThreshold = 1;
    volatile signed int neutralThrottle = 70;

    volatile signed int acceleratedThrottle = 93;

    float encoderBrakeGain = 17;
    volatile signed int deviationTargetSpeed = encoder.getCnt() - targetSpeed;

    if (targetSpeed >= 43)
    {
        neutralThrottle = 78;
        acceleratedThrottle = 100;
        largeSpeedThreshold = 6;
    }

    if (encoder.getCnt() >= targetSpeed + largeSpeedThreshold)
    {
      
      
            leftBrakeMotor = -70;
            rightBrakeMotor = -70;
        
    }
    else if (encoder.getCnt() >= targetSpeed + mediumSpeedThreshold)
    {

        if (targetSpeed >= 43)
        {
            leftBrakeMotor = 30;
            rightBrakeMotor = 30;
        }
        else{
        leftBrakeMotor = neutralThrottle - deviationTargetSpeed * encoderBrakeGain;
        rightBrakeMotor = neutralThrottle - deviationTargetSpeed * encoderBrakeGain;
        }
    }
    else if (encoder.getCnt() >= targetSpeed)
    {
        leftBrakeMotor = neutralThrottle;
        rightBrakeMotor = neutralThrottle;
    }
    else
    {
        leftBrakeMotor = acceleratedThrottle;
        rightBrakeMotor = acceleratedThrottle;
    }
}
void createHandleVal(void)
{
    volatile signed int highSpeed = 49;
    volatile signed int middleSpeed = 43;

    volatile signed int limitSpeed = 45;

    float straightCurveGain = 0.4; // 24
    float middleCurveGain = 0.73;
    float bigCurveCurveGain = 0.73;

    float middleEncoderGain = 0.27;
    float bigEncoderGain = 0;

    float middleConstEncoderGain = 29;
    float bigConstEncoderGain = 30;

    volatile signed int straightDeviation = 0;
    volatile signed int middleCurveDeviation = 7;
    volatile signed int bigCurveDeviation = 100;

    volatile signed int farTraceLine = 41;
    volatile signed int midTraceLine = 43;
    volatile signed int nearTraceLine = 45;

    float midDifferenceGain = 0.4;
    float bigDifferenceGain = 0.7;

    volatile signed int traceLine;
    if (encoder.getCnt() >= highSpeed)
    {
        traceLine = farTraceLine;
    }
    else if (encoder.getCnt() >= middleSpeed)
    {
        traceLine = midTraceLine;
    }
    else
    {
        traceLine = nearTraceLine;
    }

    if (pattern != 11)
    {
        traceLine = flagLine+3;
    }

    volatile signed int centerTraceLine = traceLine + 20;
    volatile signed int frontTraceLine = 110;

    deviationDifference = 0 /*abs(allDeviation[frontTraceLine] - allDeviation[traceLine])*/;
    // int allDeviationWa = 0;
    // for (int i = traceLine; i < frontTraceLine; i++)
    // {
    //     allDeviationWa += allDeviation[i];
    // }
    // deviationDifference = abs(allDeviationWa);
    if (pattern != 11)
    {
        handleVal = (allDeviation[traceLine] + trace_offset) * 0.5;
    }
    else
    {
        if (abs(allDeviation[traceLine]) <= straightDeviation)
        {
            handleVal = 0;
        }
        else if (abs(allDeviation[traceLine]) <= middleCurveDeviation)
        {
            // if (encoder.getCnt() >= limitSpeed)
            // {
            handleVal = (allDeviation[traceLine] + trace_offset) * straightCurveGain;

            // }
            // else
            // {
            //     handleVal = allDeviation[traceLine] * abs(middleCurveGain + middleConstEncoderGain + deviationDifference * differenceGain);
            // }
        }
        else if (deviationDifference <= bigCurveDeviation)
        {
            // if (encoder.getCnt() >= limitSpeed)
            // {
            handleVal = (allDeviation[traceLine] + trace_offset) * abs(encoder.getCnt() * middleEncoderGain + middleConstEncoderGain) / 100;
            // }
            // else
            // {
            // handleVal = allDeviation[traceLine] * middleCurveGain;
            // }
        }
        else
        {
            // if (encoder.getCnt() >= limitSpeed)
            // {
            handleVal = (allDeviation[traceLine] + trace_offset) * abs(encoder.getCnt() * bigEncoderGain + deviationDifference) / 100;
            // }
            // else
            // {
            //     handleVal = allDeviation[traceLine] * bigCurveCurveGain;
            // }
        }
    }
}

void easyCreateDeviation(int rowNum)
{

    for (int x = 0; x < IMAGE_WIDTH; x++)
    {
        easyImageData[x] = getImage(x, rowNum);
    }

    for (int x = 0; x < IMAGE_WIDTH - 1; x++)
    {
        easyDifference[x] = easyImageData[x] - easyImageData[x + 1];
    }

    int count = 0;
    for (int x = 0; x < IMAGE_WIDTH - 1; x++)
    {
        if (easyDifference[x] < 0)
        {
            easyDifferencePoint[count] = x;
            count++;
        }
    }

    int min = IMAGE_WIDTH;
    for (int c = 0; c < count; c++)
    {
        if (abs(easyDifferencePoint[c] - IMAGE_CENTER) < min)
        {
            min = abs(easyDifferencePoint[c] - IMAGE_CENTER);
            easyDeviation = easyDifferencePoint[c] - IMAGE_CENTER;
        }
    }
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//------------------------------------------------------------------//
// End of file
//------------------------------------------------------------------//
