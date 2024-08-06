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
    3040 // 3070        // 1.5ms / 0.48us - 1 = 3124   最初３０５０
// 値を足すと右　減らすと左
// 3100
#define HANDLE_STEP 18 // 1 degree value

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

//! Frame buffer stride: Frame buffer stride should be set to a multiple of 32
//! or 128
//  in accordance with the frame buffer burst transfer mode.
#define PIXEL_HW (160u) // QVGA
#define PIXEL_VW (120u) // QVGA
#define VIDEO_BUFFER_STRIDE (((PIXEL_HW * DATA_SIZE_PER_PIC) + 31u) & ~31u)
#define VIDEO_BUFFER_HEIGHT (PIXEL_VW)

#define LOG_NUM (4000u) // ログ保存数

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
unsigned char shikiichi_henkanline(int gyou, int s, int sa);

unsigned char shikiichi_henkan2(int gyou, int s, int sa, int B1, int B2, int B3,
                                int B4, int B5, int B6, int B7, int B8);
int getImage(int ix, int iy);

int getCompileYear(const char *p);
int getCompileMonth(const char *p);
int getCompileDay(const char *p);
int getCompileHour(const char *p);
int getCompilerMinute(const char *p);
int getCompilerSecond(const char *p);
unsigned long convertBCD_CharToLong(unsigned char hex);

//------------------------------------------------------------------//
// Global variable (NTSC-video)
//------------------------------------------------------------------//
static uint8_t FrameBuffer_Video_A[VIDEO_BUFFER_STRIDE * VIDEO_BUFFER_HEIGHT]
    __attribute((section("NC_BSS"), aligned(16))); // 16 bytes aligned!;
static uint8_t FrameBuffer_Video_B[VIDEO_BUFFER_STRIDE * VIDEO_BUFFER_HEIGHT]
    __attribute((section("NC_BSS"), aligned(16))); // 16 bytes aligned!;
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
volatile unsigned char sensor_bin2;

volatile unsigned char bar;
volatile int log_pattern = 901;
volatile int log_mode; // 0:待機 1:ファイルオープン
                       // 2:ログ記録中 3:ログファイルクローズ
                       // ※クローズしないとファイルが保存されない
volatile int msdError;
volatile int msd_handle, msd_l, msd_r;
signed int minG, maxG, hensaG, HG, maxgyouG, mingyouG;
// int atai[160];
static signed int Ccount = 1;
int bibunnG[160][120];
void hennsa(void);
int senn(int linegyou);
volatile int syori;
volatile int Hundle;
volatile int masugu;

volatile bool endflag = false;

volatile static int LINEgyou;
static int yakumonoLINE[30];
static int yakumonoGYOU[30];

volatile int ALL_hennsa[120];
volatile int ALL_hennsa_sei[120];
volatile int ALL_hennsa_hu[120];
volatile int LINE_30_100[70][2];
volatile int maespeed;
volatile int leftflag;
volatile int rightflag;
volatile int crossflag;
volatile int nolineflag;
volatile int barflag;
volatile int hazurereset;

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
    // storage.wait_connect(); //
    // 認識するまでここで止まってしまうのでこの命令は使えない
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

                    // signed int min, max, hensa, H, maxgyou, mingyou;
                    // int atai[160];
                    // int bibunn[160];
                    // min = 255;
                    // max = 0;
                    // for (int i = 30; i <= 129; i++)
                    // {
                    //     atai[i] = getImage(i, 118);
                    // }
                    // for (int v = 30; v <= 128; v++)
                    // {
                    //     int i, L, R;
                    //     i = v + 1;
                    //     L = atai[v];
                    //     R = atai[i];
                    //     bibunn[v] = L - R;
                    // }

                    // for (int i = 30; i <= 128; i++)
                    // {
                    //     if (max < bibunn[i])
                    //     {
                    //         max = bibunn[i]; // 8個のうち、最大を見つける
                    //         maxgyou = i;
                    //     }
                    //     if (min > bibunn[i])
                    //     {
                    //         min = bibunn[i]; // 8個のうち、最小を見つける
                    //         mingyou = i;
                    //     }
                    // }
                    // hensa = (80 - ((maxgyou + mingyou) / 2));
                    // if(){

                    // }
                    // H = hensa * 15 /20;
                    // handle(H);
                    // for (int i = 30; i <= 129; i++)
                    // {
                    //     printf("%d ", bibunn[i]);
                    // }
                    // printf("\n");
                    // hennsa(60);
                    // for (int i = 0; i <= 158; i++)
                    // {
                    //     printf("%d ", bibunnG[i]);
                    // }
                    // printf("\n");
                    // printf("max=%d   maxgyou=%d   ", maxG, maxgyouG);
                    // printf("min=%d   mingyou=%d   ", minG, mingyouG);
                    // printf("syori=%d   ", syori);
                    // printf("hensa=%d   \n", hensaG);

                    // static int H;
                    // static int HH;
                    // int O, S;
                    // static int A = 0;
                    // H = hennsa(60);
                    // // motor(100, 100);
                    // // if (abs(H) < 4)
                    // // {
                    // //     H = 0;
                    // // }

                    // if (abs(H) < 10)
                    // {
                    //     O = 1;
                    //     HH = H * -1 / 2;
                    //     handle(HH);
                    //     // handle(HH);
                    //     // motor(100, 100);
                    //     // if (H < 015
                    //     // {
                    //     //     handle((H * H * H /10000) / 2);

                    //     //     motor(100, 100);
                    //     // }
                    //     // else
                    //     // {
                    //     //     handle((H * H * H / 10000) / -2);
                    //     //     motor(100, 100);
                    //     // }
                    // }
                    // else if (abs(H) > 20)
                    // {
                    //     O = 3;

                    //     if (H < 0)
                    //     {
                    //         // HH = (H * H * H / 500) / 2;
                    //         HH = H / 10;
                    //         handle(HH);

                    //         // motor(100 - H * 40 / 5, 100);
                    //     }
                    //     else
                    //     {
                    //         // HH = (H * H * H / 500) / -2;
                    //         HH = H / 10;
                    //         handle(HH);
                    //         // motor(100, 100 - (H / 2) * 40 / 5);
                    //     }
                    // }
                    // else
                    // {
                    //     O = 2;
                    //     if (H < 0)
                    //     {
                    //         // HH = (H * H * H / 200) / 2;
                    //         HH = H / 10;
                    //         handle(HH);

                    //         // motor(100, 100);
                    //     }
                    //     else
                    //     {
                    //         // HH = (H * H * H / 200) / -2;
                    //         HH = H / 10;
                    //         handle(HH);
                    //         //  motor(100, 100);
                    //     }cx
                    // }
                    // A = A + (S - HH);
                    // S = HH;
                    // handle(A);
                    // printf("%d  %d  %d\n", O, H, HH);
                    // hennsa(60)

                    //  hennsa(60) * 28 / 100;
                    // if (abs(hennsa(60)) > 50)
                    // {
                    //     Ccount = 0;
                    // }
                    // handle(hennsa(60) * 28 / 100);
                    // // handle(0);
                    // motor(0, 0);
                    // for (int i = 0; i < 2; i++)
                    // {
                    //     printf("%d ", yakumonoGYOU[i]);
                    // }
                    // printf("\n");
                    // for (int i = 0; i < 2; i++)
                    // {
                    //     printf("%x ", yakumonoLINE[i]);
                    // }
                    // printf("\n");
                    printf("hennsa=%d", ALL_hennsa[45]);
                    printf("　　　hennsaG=%d", hensaG);
                    printf("　　　encodertotal関数=%d\n", encoder.getTotalCount());
                    printf("　　　encodergetcnt関数=%d\n", encoder.getCnt());

                    // 補正値で表示 しきい値180以上を"1" 180を変えると、しきい値が変わる
                    // printf("shikii chi 180\r\n");
                    // for (y = 0; y < 30; y++)
                    // {
                    //     printf("%3d:%08ld ", y + 0,
                    //     convertBCD_CharToLong(shikiichi_henkan(y + 0, 180, 8)));
                    //     printf("%3d:%08ld ", y + 30,
                    //     convertBCD_CharToLong(shikiichi_henkan(y + 30, 180, 8)));
                    //     printf("%3d:%08ld ", y + 60,
                    //     convertBCD_CharToLong(shikiichi_henkan(y + 60, 180, 8)));
                    //     printf("%3d:%08ld ", y + 90,
                    //     convertBCD_CharToLong(shikiichi_henkan(y + 90, 180, 8)));
                    //     printf("\r\n");
                    // }
                    // printf("\033[H");
                    break;

                case 2:
                    // 補正値で表示 しきい値120以上を"1" 180を変えると、しきい値が変わる
                    printf("shikii chi 120\r\n");
                    for (y = 0; y < 30; y++)
                    {
                        printf("%3d:%08ld ", y + 0,
                               convertBCD_CharToLong(shikiichi_henkan(y + 0, 120, 8)));
                        printf("%3d:%08ld ", y + 30,
                               convertBCD_CharToLong(shikiichi_henkan(y + 30, 120, 8)));
                        printf("%3d:%08ld ", y + 60,
                               convertBCD_CharToLong(shikiichi_henkan(y + 60, 120, 8)));
                        printf("%3d:%08ld ", y + 90,
                               convertBCD_CharToLong(shikiichi_henkan(y + 90, 120, 8)));
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
                    printf("shi 0         0         0         0         0         0      "
                           "   0         0         0         0         1         1       "
                           "  1         1         1         1        1\r\n");
                    printf("kii 0         1         2         3         4         5      "
                           "   6         7         8         9         0         1       "
                           "  2         3         4         5        5\r\n");
                    printf("180 "
                           "0123456789012345678901234567890123456789012345678901234567890"
                           "1234567890123456789012345678901234567890123456789012345678901"
                           "23456789012345678901234567890123456789\r\n");
                    for (y = 0; y < 120; y += 2)
                    {
                        printf("%03d:", y);
                        for (x = 0; x < 160; x++)
                        {
                            c = getImage(x, y) >= 180 ? 1
                                                      : 0; // 180を変えるとしきい値が変わる
                            if (y == 60 && (x == 31 || x == 43 || x == 54 || x == 71 ||
                                            x == 88 || x == 105 || x == 116 || x == 128))
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

                case 4:
                    // 60～119行を表示(しきい値120以上を"1"とする)
                    printf("shi 0         0         0         0         0         0      "
                           "   0         0         0         0         1         1       "
                           "  1         1         1         1        1\r\n");
                    printf("kii 0         1         2         3         4         5      "
                           "   6         7         8         9         0         1       "
                           "  2         3         4         5        5\r\n");
                    printf("120 "
                           "0123456789012345678901234567890123456789012345678901234567890"
                           "1234567890123456789012345678901234567890123456789012345678901"
                           "23456789012345678901234567890123456789\r\n");
                    for (y = 60; y < 120; y++)
                    {
                        printf("%03d:", y);
                        for (x = 0; x < 160; x++)
                        {
                            c = getImage(x, y) >= 120 ? 1
                                                      : 0; // 120を変えるとしきい値が変わる
                            if ((y == 110 || y == 111 || y == 112 || y == 113 || y == 114) &&
                                (x == 31 || x == 128))
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
    // if (endflag == true)
    // {
    while (1)
    {
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
                fprintf(fp, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
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
                        log_data[i].total);

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
    // }
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
    error = Display.Graphics_Irq_Handler_Set(DisplayBase::INT_TYPE_S0_VI_VSYNC, 0,
                                             IntCallbackFunc_Vsync);
    if (error != DisplayBase::GRAPHICS_OK)
    {
        printf("Line %d, error %d\n", __LINE__, error);
        while (1)
            ;
    }

    // Video capture setting (progressive form fixed)
    error = Display.Video_Write_Setting(
        VIDEO_INPUT_CH, DisplayBase::COL_SYS_NTSC_358, write_buff_addr,
        VIDEO_BUFFER_STRIDE, DisplayBase::VIDEO_FORMAT_YCBCR422,
        DisplayBase::WR_RD_WRSWA_32_16BIT, PIXEL_VW, PIXEL_HW);
    if (error != DisplayBase::GRAPHICS_OK)
    {
        printf("Line %d, error %d\n", __LINE__, error);
        while (1)
            ;
    }

    // Interrupt callback function setting (Field end signal for recording
    // function in scaler 0)
    error = Display.Graphics_Irq_Handler_Set(VIDEO_INT_TYPE, 0,
                                             IntCallbackFunc_Vfield);
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
    error = Display.Video_Write_Change(VIDEO_INPUT_CH, write_buff_addr,
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
    volatile static int counter = 0; // Only variable for image process
    volatile unsigned char b;
    volatile static int Ecount;
    volatile static int stahen = 0;
    volatile static int stahen1 = 0;
    volatile static int treline;

    volatile static int kabutreline = 0;
    volatile static int beforspeed;
    volatile static int speedsa;

    cnt1++;
    cnt_msdwritetime++;
    cnt_printf++;
    cnt_msd++;
    cnt_debug++;
    encoder.kyoriupdate();
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
        ImageCopy(write_buff_addr, PIXEL_HW, PIXEL_VW, ImageData_A,
                  vfield_count2); //  0 - 59行を変換
        break;
    case 1:
        ImageCopy(write_buff_addr, PIXEL_HW, PIXEL_VW, ImageData_A,
                  vfield_count2); // 60 - 119行を変換
        break;
    case 2:
        Extraction_Brightness(ImageData_A, PIXEL_HW, PIXEL_VW, ImageData_B,
                              vfield_count2);
        break;
    case 3:
        Extraction_Brightness(ImageData_A, PIXEL_HW, PIXEL_VW, ImageData_B,
                              vfield_count2);

        sensor_bin = shikiichi_henkan(
            60, 250, 8); //  60行目 しきい値180 隣同士の差分が8以下なら0x00にする

        sensor_bin2 = shikiichi_henkanline(
            90, 250, 8); //  60行目 しきい値180 隣同士の差分が8以下なら0x00にする
        // スタートバー検出 bit7とbit0のみ見る
        bar = (shikiichi_henkan(110, 120, 8) | shikiichi_henkan(111, 120, 8) |
               shikiichi_henkan(112, 120, 8) | shikiichi_henkan(113, 120, 8) |
               shikiichi_henkan(114, 120, 8)) &
              0x81; // 1000 0001　

        // signed int min, max, hensa, H, maxgyou, mingyou, sotosenngyou, sotosenn;
        // // int atai[160];
        // // int bibunn[160];
        // int waku[81];
        // min = 255;
        // max = 0;
        // sotosenn = 255;
        // for (int i = 30; i <= 129; i++)
        // {
        //     atai[i] = getImage(i, 60);
        // }
        // for (int i = 0; i <= 80; i++)
        // {
        //     waku[i] = atai[i] - atai[i - 1];
        // }
        // for (int i = 0; i <= 80; i++)
        // {
        //     if (sotosenn > waku[i])
        //     {
        //         sotosenn = waku[i]; // 8個のうち、最小を見つける
        //         sotosenngyou = i;
        //     }
        // }
        // for (int v = sotosenngyou + 5; v <= sotosenngyou + 90; v++)
        // {

        //     bibunn[v] = atai[v] - atai[v + 1];
        // }

        // for (int i = sotosenngyou + 5; i <= sotosenngyou + 90; i++)
        // {
        //     if (max < bibunn[i])
        //     {
        //         max = bibunn[i]; // 8個のうち、最大を見つける
        //         maxgyou = i;
        //     }
        //     if (min > bibunn[i])
        //     {
        //         min = bibunn[i]; // 8個のうち、最小を見つける
        //         mingyou = i;
        //     }
        // }
        // hensa = (80 - ((maxgyou + mingyou) / 2));
        // if (hensa >= -5 && hensa <= 5)
        // {
        //     hensa = 0;
        // }

        // max2 = max;
        // min2 = min;
        // maxgyou2 = maxgyou;

        // mingyou2 = mingyou;
        // hensa2 = hensa;
        // H2 = H;

        break;
    case 5:

        // MTU2TCNT_0 = 0;
        if (encoder.getCnt() >= 42)
        {
            treline = 31;
        }

        else if (encoder.getCnt() >= 40)
        {
            treline = 35;
        }

        else
        {
            treline = 39;
        }
        hennsa();
        senn(51);
        if (encoder.getCnt() >= 41)
        {
            kabutreline = 50;
        }

        else if (encoder.getCnt() >= 35)
        {
            kabutreline = 50;
        }

        else
        {
            kabutreline = 50;
        }

        Hundle = ALL_hennsa[kabutreline] - 0;

        stahen = ALL_hennsa[70] - 0;
        stahen1 = ALL_hennsa[70] - 0;

        masugu = ALL_hennsa[treline] - 0;
        // senn();
        break;
    case 6:
        // for (int i2 = 0; i2 <= 159; i2++)
        // {

        //     printf("%d", getImage(i2, 60));
        // }
        // printf("\n");
        //  Hundle = hennsa(60);
        MTU2TCNT_0 = 0;
        break;
    case 10:
        maespeed = encoder.getCnt();
        encoder.update();
        speedsa = encoder.getCnt() - beforspeed;
        beforspeed = encoder.getCnt();
        if (endflag == false)
        {
            // ログ（RAM）記録
            log_data[log_no].cnt_msdwritetime = cnt_msdwritetime;
            log_data[log_no].pattern = pattern;
            log_data[log_no].convertBCD = sensor_inp(0xff);
            log_data[log_no].handle = msd_handle;
            log_data[log_no].hennsa = ALL_hennsa[38];
            log_data[log_no].hennsa = ALL_hennsa[38];
            log_data[log_no].encoder = encoder.getCnt();
            log_data[log_no].motorL = msd_l;
            log_data[log_no].motorR = msd_r;

            log_data[log_no].flagL = leftflag;
            log_data[log_no].flagK = crossflag;
            log_data[log_no].flagR = rightflag;
            log_data[log_no].flagS = nolineflag;

            log_data[log_no].total = encoder.getTotalCount();
            log_data[log_no].total = encoder.getTotalCount();

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

    // モータドライブ基板のLED2個は、スタートバーの反応チェック用
    // スタート時にLEDが点灯するようにセットする
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
        // if (pushsw_get() == 1)
        // {
        //     pattern = 1;
        //     log_mode = 1; // ログファイルオープン
        //     cnt1 = 0;
        //     break;
        // }
        if (barflag == 0) // バースタート
        // if (crossflag == 0)
        {
            if (pushsw_get() == 1)
            {
                pattern = 1;
                log_mode = 1; // ログファイルオープン
                cnt1 = 0;
                cnt1 = 0;
                break;
            }
            led_m(50, 0, 1, 0); // スタートバーセットOK状態→緑色点灯
        }
        else
        {
            if (pushsw_get() == 1) // ba-nasi
            {
                pattern = 4;
                log_mode = 1; // ログファイルオープン
                cnt1 = 0;
                encoder.clear();
                break;
            }
            led_m(10, 1, 0, 0); // スタートバーセットNG状態→赤色点灯
        }
        break;

    case 1:
        // スタートバーが開いたかチェック
        if (cnt1 >= 2000)
        {
            motor(20, 20);
        }
        // handle(H * (25) / 100);}

        //handle((stahen1) / 2.5);
        handle(ALL_hennsa[60] * 50 / 100);


        // if (bar == 1)
        if (barflag == 1)
       // if (pushsw_get() == 1) // ba-nasi

        {
            // スタート！！
            led_m(0, 0, 0, 0);
            log_mode = 2; // ログ記録中
            cnt_msdwritetime = 0;
            pattern = 2;
            cnt1 = 0;
            encoder.clear();
            break;
        }
        led_m(10, 1, 1, 0);
        break;

    case 2:
        handle(0);

        // if (cnt1 >= 50)
        // {
        //     motor(-20, -20);
        // }
        // else{
        motor(0, 0);

       // }
       // if (cnt1 >= 50)
        // if (crossflag == 0)
        // bar == 0x00
        if (barflag == 0&&cnt1 >= 1000)
       // if (pushsw_get() == 1) // ba-nasi

        {
            pattern = 3;
        }

        // pattern = 11;
        Ccount = 1;
        break;
    case 3:
        // handle(0);
        // handle(Hundle * ((abs(Hundle) * 15) / 10 + (encoder.getCnt() * 35) / 10) / 680);
        handle(ALL_hennsa[60] * 50 / 100);
        motor(80, 80);
       // if (encoder.getTotalCount() >= 0)
            if (encoder.getTotalCount() >= 100)
            {
                pattern = 10;
                cnt1 = 0;
            }
        break;
    case 4:
        if (barflag == 0)
        {
            pattern = 10;
        }
        break;

   
    case 10:
        // スタートバーが開いたかチェック
        if (bar == 0x00)
        {
            // スタート！！
            led_m(0, 0, 0, 0);
            //log_mode = 2; // ログ記録中
            cnt_msdwritetime = 0;
            pattern = 11;
            cnt1 = 0;
            break;
        }
        led_m(10, 1, 1, 0);
        break;

    case 11:
        // 通常トレース
        led_m(10, 1, 1, 1);
        volatile static int H;
        volatile static int HH, L, R;
        volatile int O;
        volatile int LINE;

        H = Hundle;
        // while(1){
        //     if(cnt1<1000){
        //         handle(-28);
        //     }

        //     else if(cnt1<2000){
        //         handle(28);
        //     }

        //     else{
        //       cnt1=0;
        //     }

        // }

        //if (abs(masugu) < 15 && encoder.getTotalCount() >= 500)     // 400
            if (abs(masugu) < 15 && encoder.getTotalCount() >= 400) // 400
            {

                if (crossflag == 1)
                { // クロスラインチェック
                    pattern = 21;
                    break;
                }
                if (rightflag == 1)
                { // 右ハーフラインチェック
                    pattern = 51;
                    break;
                }
                if (leftflag == 1)
                { // 左ハーフラインチェック
                    pattern = 61;
                    break;
                }
            }

        L = 100 - speedsa * 13;
        R = 100 - speedsa * 13;
        if (L > 100)
        {
            L = 100;
        }
        if (R > 100)
        {
            R = 100;
        }
        // if (encoder.getCnt() >= 43)
        // {
        //     L = 85;
        //     R = 85;
        // }
        if (encoder.getCnt() >= 47)
        {
            L = 60;
            R = 60;
            if (speedsa > 0)
            {
                L = -0;
                R = -0;
            }
        }
        // if (encoder.getCnt() >= 47)
        // {
        //     L = 30;
        //     R = 30;
        // }

        if (abs(masugu) > 0)
        {
            HH = (masugu) * 40 / 100;
            // HH = 0;
            //   HH = masugu * ((abs(masugu) * 20) / 10 + (encoder.getCnt() * 10) / 10) / 120;
        }
        else if (abs(masugu) > 50)
        {
            HH = (masugu) * 69 / 100; // 69
            if (encoder.getCnt() >= 42)
            {
               // HH = (masugu) * 73 / 100; // 69
                HH = (masugu) * (encoder.getCnt() * 1.3 + 15) / 100;

                // if(speedsa > 0){
                //     L = 30;
                //     R = 30;
                // }

               
            }
           
        }
        else if (abs(masugu) > 10)
        {
            HH = (masugu) * 68 / 100; // 69
            if (encoder.getCnt() >= 42)
            {
               // HH = (masugu) * 73 / 100; // 69
                HH = (masugu) * (encoder.getCnt() * 1.1 + 15) / 100;

                // if(speedsa > 0){
                //     L = 30;
                //     R = 30;
                // }

               
            }
           
        }
        else
        {
            HH = 0;
            HH = 0;
        }

        motor(L, R);
        handle(HH);

        if (abs(masugu) > 100)
        {

            pattern = 12;

            break;
        }
        if (encoder.getCourseCount() >= 63000 * 3)
        {
            pattern = 101;
        }
        break;

    case 12:

        L = 100 - speedsa * 13;
        R = 100 - speedsa * 13;
        if (L > 100)
        {
            L = 100;
        }
        if (R > 100)
        {
            R = 100;
        }
        if (encoder.getCnt() >= 40)
        {
            L = 80;
            R = 80;
        }
        if (abs(masugu) < 10)
        {
            HH = (masugu) * 40 / 100;
            // HH = 0;
            //   HH = masugu * ((abs(masugu) * 20) / 10 + (encoder.getCnt() * 10) / 10) / 120;
        }
        else
        {
            HH = (Hundle) * 50 / 100;
            if (encoder.getCnt() >= 43)
            {
                HH = (Hundle) * (encoder.getCnt() + 5) / 100;
            }
        }
        // HH = H * ((abs(Hundle) * 10) / 10 + (encoder.getCnt() * 45) / 7) / 720;

        // HH = H * ((abs(H) * 35) / 10 + (encoder.getCnt() * 10) / 10) / 120;

        // if (encoder.getCnt() >= 35)
        // {
        //     L = 90;
        //     R = 90;
        // }
        // if (encoder.getCnt() >= 40)
        // {
        //     L = 60;
        //     R = 60;
        // }
        // if (encoder.getCnt() >= 45)
        // {
        //     L = 30;
        //     R = 30;
        // }

        if (H < 0)
        {
            // H -= (6);
            // R = 60;
            // L = 60;
        }

        else
        {
            // H += (6);
            // R = 60;
            // L = 60;
        }

        if (abs(masugu) < 15 && abs(Hundle) < 15)
        {

            pattern = 11;

            break;
        }

        motor(L, R);
        handle(HH);

        motor(L, R);
        handle(HH);
        break;

    case 21:
        // クロスライン検出時の処理
        led_m(10, 1, 0, 1);
        if (encoder.getCnt() >= 33)
        {
            L = -100;
            R = -100;
        }

        else if (encoder.getCnt() >= 27)
        {
            L = 10;
            R = 10;
        }

        else if (encoder.getCnt() >= 25)
        {
            L = 60;
            R = 60;
        }

        else
        {
            L = 100;
            R = 100;
        }

        motor(L, R);
        Ccount = 1;
        pattern = 22;
        hazurereset=1;

        encoder.clear();

        handle(0);
        // handle((ALL_hennsa[40] + 10) * 25 / 100);

        // handle((ALL_hennsa[55] + 9) * 35 / 100);

        // handle((ALL_hennsa[40] + 10) * 25 / 100);

        // handle((ALL_hennsa[55] + 9) * 35 / 100);

        break;

    case 22:
        // クロスラインを読み飛ばす
        led_m(50, 1, 0, 1);

        // handle(ALL_hennsa[80] * 10 / 100);
        // handle(ALL_hennsa[80] * 10 / 100);
        handle(0);
        // handle((ALL_hennsa[40] + 10) * 25 / 100);

        // handle((ALL_hennsa[55] + 9) * 35 / 100);

        // handle((ALL_hennsa[40] + 10) * 25 / 100);

        // handle((ALL_hennsa[55] + 9) * 35 / 100);

        if (encoder.getCnt() >= 33)
        {
            L = -100;
            R = -100;
        }

        else if (encoder.getCnt() >= 27)
        {
            L = 10;
            R = 10;
        }

        else if (encoder.getCnt() >= 25)
        {
            L = 60;
            R = 60;
        }

        else
        {
            L = 100;
            R = 100;
        }
        motor(L, R);

    //   if (crossflag == false && rightflag == false && leftflag == false && encoder.getTotalCount() > 200)
            // if (encoder.getTotalCount() > 180)
        if (crossflag == false && rightflag == false && leftflag == false && encoder.getTotalCount() > 300)
            // if (encoder.getTotalCount() > 180)
            {

                pattern = 23;
                hazurereset=1;

                Ccount = 1;
                encoder.clear();
            }

        break;

    case 23:
        // クロスライン後のトレース、クランク検出
        led_m(1, 1, 0, 1);
        static int na;
        static int ni;
        // if(abs(ALL_hennsa[40])>=10){
        //     pattern=11;
        //     break;
        // }

        //  if (check_leftline() == 1 || senn() == 0x10)
        if (leftflag == 1)
        // if (check_leftline() == 1)
        {
            // 左クランクと判断→左クランククリア処理へ
            led_m(100, 1, 0, 1);

            handle(41); // 25

            // motor(80, 0-encoder.getCnt()*2-encoder.getCnt()*2);
            encoder.setvalue(0);
            // motor(80, 0-encoder.getCnt()*2-encoder.getCnt()*2);

            // motor(80 - encoder.getCnt()/2, 80);
            motor(40, 80); // 60,80kami

            pattern = 31;
            // pattern=32;
            Ccount = 0;
            encoder.clear();
            break;
        }

        //    if (check_rightline() == 1 || senn() == 0x01)
        else if (rightflag == 1)
        // if (check_rightline() == 1)
        {
            // 右クランクと判断→右クランククリア処理へ
            led_m(100, 0, 0, 1);

            handle(-41); // 25

            encoder.setvalue(0);
            // motor(80, 0-encoder.getCnt()*2-encoder.getCnt()*2);

            // motor(80 - encoder.getCnt()/2, 80);
            motor(80, 40); // 60,80kami

            pattern = 41;
            // pattern=32;
            Ccount = 0;
            encoder.clear();
            break;
        }
        // H = hennsa(60);
        // O = 1;
        handle(ALL_hennsa[60] * 50 / 100);
        //  handle(0);

        if (encoder.getCnt() >= 33)
        {
            L = -100;
            R = -100;
        }

        else if (encoder.getCnt() >= 27)
        {
            L = 10;
            R = 10;
        }

        else if (encoder.getCnt() >= 25)
        {
            L = 60;
            R = 60;
        }

        else
        {
            L = 100;
            R = 100;
        }

        motor(L, R);

        break;

    case 31:
        // 左クランククリア処理　安定するまで少し待つ
        led_m(100, 1, 1, 0);
        // if (cnt1 >= 400 - encoder.getCnt() * 5)
        // {
        //     pattern = 42;
        //     cnt1 = 0;
        // }
        // if (nolineflag == 0)
        if (encoder.getTotalCount() >= 470)
        // if (nolineflag == 0)
        {
            pattern = 11;
            hazurereset=1;

            //  cnt1 = 0;
            pattern = 11;
            //  cnt1 = 0;
        }
        //   if (abs(hennsa(60)) <= 10 && cnt1 > 10)
        // if (senn() == 0x10)
        // {
        //     pattern = 11;
        //     Ccount = 0;
        // }
        break;

    case 32:
        // 左クランククリア処理　曲げ終わりのチェック
        // if (sensor_inp(MASK3_3) == 0x60)
        // {
        //     led_m(100, 0, 0, 0);
        //     pattern = 11;
        //     encoder.clear();
        //     cnt1 = 0;
        // }
        // if (sensor_inp(MASK3_3) == 0x60)
        // {
        //     led_m(100, 0, 0, 0);
        //     pattern = 11;
        //     encoder.clear();
        //     cnt1 = 0;
        // }

        // if (abs(Hundle) <= 20)
        // {
        //     led_m(100, 0, 0, 0);
        //     pattern = 11;
        //     encoder.clear();

        //     cnt1 = 0;
        // }
        if (nolineflag == 0)
        {
            led_m(100, 0, 0, 0);
            pattern = 11;
            encoder.clear();

            // cnt1 = 0;
            // cnt1 = 0;
        }

        // // 外側の白線を読んだら
        // if (sensor_inp(MASK3_3) == 0x07)
        // {
        //     pattern = 33;
        //     break;
        // }

        break;

    case 33:
        // 左クランククリア処理　外側の白線と見間違わないようにする
        b = sensor_inp(MASK3_3);
        if (b == 0x83 || b == 0x81 || b == 0xc1)
        {
            pattern = 32;
        }
        if (abs(Hundle) <= 20)
        {
            led_m(100, 0, 0, 0);
            pattern = 11;
            // cnt1 = 0;
            // cnt1 = 0;
        }
        break;

    case 41:
        // 右クランククリア処理　安定するまで少し待つ
        led_m(100, 0, 1, 1);
        // if (cnt1 >= 400 - encoder.getCnt() * 5)
        // {
        //     pattern = 42;
        //     cnt1 = 0;
        // }
        // if (nolineflag == 0)

        if (encoder.getTotalCount() >= 470)
        // if (nolineflag == 0)

        {
            pattern = 11;
            hazurereset=1;

            // cnt1 = 0;
            pattern = 11;
            // cnt1 = 0;
        }

        // if (abs(Hundle) <= 20)
        // {
        //     led_m(100, 0, 0, 0);
        //     pattern = 11;
        //     cnt1 = 0;
        // }
        //   if (abs(hennsa(60)) <= 10 && cnt1 > 10)
        // if (senn() == 0x10)
        // {
        //     pattern = 11;
        //     Ccount = 0;
        // }
        break;

    case 42:
        // 右クランククリア処理　曲げ終わりのチェック
        // if (sensor_inp(MASK3_3) == 0x06)
        // { // ○○○○　○●●○
        //     led_m(100, 0, 0, 0);
        //     pattern = 11;
        //     encoder.clear();
        // if (sensor_inp(MASK3_3) == 0x06)
        // { // ○○○○　○●●○
        //     led_m(100, 0, 0, 0);
        //     pattern = 11;
        //     encoder.clear();

        //     cnt1 = 0;
        // }
        //     cnt1 = 0;
        // }
        // if (abs(Hundle) <= 20)
        // {
        //     led_m(100, 0, 0, 0);
        //     pattern = 11;
        //     encoder.clear();

        //     cnt1 = 0;
        // }
        if (nolineflag == 0)
        {
            led_m(100, 0, 0, 0);
            pattern = 11;
            encoder.clear();

            // cnt1 = 0;
            // cnt1 = 0;
        }

        // 外側の白線を読んだら
        // if (sensor_inp(MASK3_3) == 0xe0)
        // { // ●●○○ ○○○○
        //     pattern = 43;
        //     break;
        // }
        // break;

    case 43:
        // 右クランククリア処理　外側の白線と見間違わないようにする
        // b = sensor_inp(MASK3_3);
        // if (b == 0xc1 || b == 0x81 || b == 0x83)
        // {
        //     pattern = 42;
        // }
        if (abs(Hundle) <= 20)
        {
            led_m(100, 0, 0, 0);
            pattern = 11;
            // cnt1 = 0;
            // cnt1 = 0;
        }
        break;

    case 51:
        // 右ハーフライン検出時の処理
        led_m(100, 0, 1, 0);
        // handle(ALL_hennsa[100] * 30 / 100);
        // handle(0);
        handle((ALL_hennsa[55] - 9) * 35 / 100);
        // handle(0);

        // if (encoder.getCnt() < 28)
        // {
        //     L = 60;
        //     R = 60;
        // }
        // if (encoder.getCnt() < 28)
        // {
        //     L = 60;
        //     R = 60;
        // }

        // else if (encoder.getCnt() < 32)
        // {
        //     L = 50;
        //     R = 50;
        // }
        // else if (encoder.getCnt() < 32)
        // {
        //     L = 50;
        //     R = 50;
        // }

        // //   else if (encoder.getCnt() < 40)
        // else
        // {
        L = 60;
        R = 60;
        // }
        // //   else if (encoder.getCnt() < 40)
        // else
        // {

        // }
        motor(L, R);
        pattern = 52;
        // cnt1 = 0;
        // cnt1 = 0;
        encoder.clear();
        // handle((ALL_hennsa[55] + 9 - 9) * 35 / 100);

        // handle((ALL_hennsa[55] + 9 - 9) * 35 / 100);

        // if (check_leftline() == 1 || check_crossline() == 1)
        if (leftflag == 1 || crossflag == 1)
        {
            pattern = 21;
            break;
        }

        break;

    case 52:

        // 右ハーフラインを読み飛ばす
        if (rightflag == false && encoder.getTotalCount() > 200)
        {
            pattern = 53;
            // cnt1 = 0;
            // cnt1 = 0;
            encoder.clear();
        }
        // if (check_leftline() == 1 || check_crossline() == 1)
        if (leftflag == 1 || crossflag == 1)

        {
            pattern = 21;
            break;
        }
        /// handle(ALL_hennsa[100] * 30 / 100);
        //        handle(0);

        // handle((ALL_hennsa[55] + 9 - 9) * 35 / 100);

        /// handle(ALL_hennsa[100] * 30 / 100);
        //        handle(0);
        handle((H - 9) * 35 / 100);

        // handle((ALL_hennsa[55] + 9 - 9) * 35 / 100);

        // handle(ALL_hennsa[80] * 10 / 100);
        //  handle(ALL_hennsa[100] * 10 / 100);
        //  handle((stahen + 2) / 2.5);
        //  handle(ALL_hennsa[100] * 10 / 100);
        //  handle((stahen + 2) / 2.5);

        // if (encoder.getCnt() < 32)
        // {
        //     L = 80;
        //     R = 80;
        // }
        // if (encoder.getCnt() < 32)
        // {
        //     L = 80;
        //     R = 80;
        // }

        // else if (encoder.getCnt() < 35)
        // {
        //     L = 50;
        //     R = 50;
        // }
        // else if (encoder.getCnt() < 35)
        // {
        //     L = 50;
        //     R = 50;
        // }

        // }
        // //   else if (encoder.getCnt() < 40)
        // else
        // {
        L = 60;
        R = 60;
        // }
        motor(L, R);

        break;

    case 53:
        // 右ハーフライン後のトレース、レーンチェンジ

        // if (sensor_inp(0x3c) == 0x00)
        //  if (getImage(80, 60) <= 200)
        if (nolineflag == 1)
        {
            led_m(50, 0, 0, 1);

            // if (shikiichi_henkan2(60, 180, 8, 70, 72, 74, 78, 80, 82, 84, 86) == 0x00)
            // {
            handle(-10);
            motor(80, 100);
            pattern = 54;
            encoder.clear();
            cnt1 = 0;
            cnt1 = 0;
            break;
        }
        // handle(Hundle * 20 / 100);
        handle((H) * 35 / 100);
        if (encoder.getCnt() > 25)
        {
            L = -30;
            R = -30;
        }

        else
        {
            L = 60;
            R = 60;
        }
        L = 60;
        R = 60;
        // }
        motor(L, R);

        break;

    case 54:

        // 右レーンチェンジ終了のチェック
        b = sensor_inp(0x3c);
        //
        led_m(50, 1, 0, 0);

        // if (getImage(80, 80) >= 200)
        // if (b > 1 && encoder.getTotalCount() >= 300)
        if (encoder.getTotalCount() >= 250)

        {
            led_m(100, 0, 0, 0);
            pattern = 55;
            // cnt1 = 0;
            encoder.clear();
        }
        // if (abs(hennsa(60)) <= 20)
        // {
        //   led_m(100, 0, 0, 0);
        //   pattern = 11;
        //   cnt1 = 0;
        // }
        break;
    case 55:

        // 右レーンチェンジ終了のチェック
        b = sensor_inp(0x3c);
        //
        led_m(50, 1, 0, 0);
        handle((encoder.getCnt() / 8));
        //

        motor(100, 100);

        // if (getImage(80, 80) >= 200)
        // if (b > 1 && encoder.getTotalCount() >= 300)
        if (nolineflag == 0)

        {
            led_m(100, 0, 0, 0);
            pattern = 11;
            // cnt1 = 0;
            // cnt1 = 0;
            encoder.clear();
        }
        // if (abs(hennsa(60)) <= 20)
        // {
        //   led_m(100, 0, 0, 0);
        //   pattern = 11;
        //   cnt1 = 0;
        // }
        break;

    case 61:
        // 左ハーフライン検出時の処理

        led_m(100, 0, 1, 0);
        // handle(ALL_hennsa[100] * 30 / 100);
        //        handle(0);
        handle((H + 9) * (35) / 100);
        //        handle(0);

        // if (encoder.getCnt() < 28)
        // {
        //     L = 80;
        //     R = 80;
        // }
        // if (encoder.getCnt() < 28)
        // {
        //     L = 80;
        //     R = 80;
        // }

        // else if (encoder.getCnt() < 32)
        // {
        //     L = 60;
        //     R = 60;
        // }

        // //   else if (encoder.getCnt() < 40)
        // else
        // {

        if (encoder.getCnt() >= 40)
        {
            L = 0;
            R = 0;
        }

        else if (encoder.getCnt() >= 36)
        {
            L = 10;
            R = 10;
        }

        else if (encoder.getCnt() >= 33)
        {
            L = 80;
            R = 80;
        }

        else
        {
            L = 100;
            R = 100;
        }
        // }

        motor(L, R);
        pattern = 62;
        hazurereset=1;

        // cnt1 = 0;
        // cnt1 = 0;
        encoder.clear();
        // handle((ALL_hennsa[55] + 9 + 9) * (35) / 100);

        // handle((ALL_hennsa[55] + 9 + 9) * (35) / 100);

        // if (check_leftline() == 1 || check_crossline() == 1)
        if (rightflag == 1 || crossflag == 1)
        {
            pattern = 21;
            break;
        }

        break;

    case 62:
        // 左ハーフラインを読み飛ばす

        if (leftflag == false && encoder.getTotalCount() > 200)
        {
            pattern = 63;
            hazurereset=1;

            // cnt1 = 0;
            // cnt1 = 0;
            encoder.clear();
        }
        // if (check_leftline() == 1 || check_crossline() == 1)
        if (rightflag == 1 || crossflag == 1)

        {
            pattern = 21;
            break;
        }
        // handle(ALL_hennsa[100] * 30 / 100);
        // handle(0);
        handle((H + 9) * (35) / 100);

        if (encoder.getCnt() >= 40)
        {
            L = 0;
            R = 0;
        }

        else if (encoder.getCnt() >= 36)
        {
            L = 10;
            R = 10;
        }

        else if (encoder.getCnt() >= 33)
        {
            L = 80;
            R = 80;
        }

        else
        {
            L = 100;
            R = 100;
        }
        motor(L, R);

        break;

  case 63:

        // 左ハーフライン後のトレース、レーンチェンジ
        // if (sensor_inp(0x3c) == 0x00)
        // if (getImage(80, 60) <= 200)
        if (nolineflag == 1)
        {
            led_m(50, 0, 0, 1);

            // if (shikiichi_henkan2(60, 180, 8, 70, 72, 74, 78, 80, 82, 84, 86) == 0x00)
            // {
            handle(0);
            motor(60, 60);
            pattern = 640;
            encoder.clear();
            cnt1 = 0;
            cnt1 = 0;
            break;
        }
        handle((H) * (35) / 100);
        // handle(ALL_hennsa[55] * 30 / 100);
        if (encoder.getCnt() >= 30)
        {
            L = -100;
            R = -100;
        }

        else if (encoder.getCnt() >= 27)
        {
            L = 10;
            R = 10;
        }

        else if (encoder.getCnt() >= 25)
        {
            L = 60;
            R = 60;
        }

        else
        {
            L = 100;
            R = 100;
        }

        motor(L, R);

        break;

    case 640:

        if (encoder.getTotalCount() > 100)
        {
             handle(30);
            motor(30, 50);
            pattern = 64;
            break;
        }
        break;

    case 64:
        b = sensor_inp(0x3c);
        //
        led_m(50, 1, 0, 0);

        // if (getImage(80, 80) >= 200)
        // if (b > 1 && encoder.getTotalCount() >= 300)
        // if (encoder.getTotalCount() >= 300)
        if (/*encoder.getTotalCount() >= 400*/ nolineflag == 0)

        {
            led_m(100, 0, 0, 0);
            pattern = 65;
            // cnt1 = 0;
            hazurereset=1;
            encoder.clear();
        }
        break;

    case 65:
        b = sensor_inp(0x3c);
        //
        led_m(50, 1, 0, 0);
        handle(0);
        //

        motor(40, 40);
        // if (getImage(80, 80) >= 200)
        // if (b > 1 && encoder.getTotalCount() >= 300)
        if (encoder.getTotalCount() >= 10)

        {
            led_m(100, 0, 0, 0);
            pattern = 67;
            // cnt1 = 0;
            encoder.clear();
        }
        break;

    case 67:
        b = sensor_inp(0x3c);
        //
        led_m(50, 1, 0, 0);
        handle(-20);
        //

        motor(40, 20);
        // if (getImage(80, 80) >= 200)
        // if (b > 1 && encoder.getTotalCount() >= 300)
        if (encoder.getTotalCount() >= 300)

        {
            led_m(100, 0, 0, 0);
            pattern = 11;
            // cnt1 = 0;
            hazurereset=1;

            encoder.setvalue(-400);
        }
        break;

    case 101:
        // 終了　ログ保存中など
        led_m(100, 0, 1, 1);
        handle((stahen + 6) / 2.5);
        endflag = true;
        // interrput.detach();
        motor(0, 0);
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
    /*
    sw_data = dipsw_get() + 5;
    accele_l = ( accele_l * sw_data ) / 20;
    accele_r = ( accele_r * sw_data ) / 20;
    */

    msd_l = accele_l;
    msd_r = accele_r;

    // Left Motor Control

    // if (accele_l == 100)
    // {
    //     // forward
    //     Left_motor_signal = 0;
    //     MTU2TGRC_4 = (long)(MOTOR_PWM_CYCLE);
    // }

    // else
    if (accele_l >= 0)
    {
        // forward
        Left_motor_signal = 0;
        MTU2TGRC_4 = (long)(MOTOR_PWM_CYCLE - 1) * accele_l / 100;
    }
    else
    {
        // reverse
        Left_motor_signal = 1;
        MTU2TGRC_4 = (long)(MOTOR_PWM_CYCLE - 1) * (-accele_l) / 100;
    }

    // Right Motor Control
    // if (accele_r == 100)
    // {
    //     // forward
    //     Right_motor_signal = 0;
    //     MTU2TGRD_4 = (long)(MOTOR_PWM_CYCLE + 2);
    // }

    // else
    if (accele_r >= 0)
    {
        // forward
        Right_motor_signal = 1;
        Right_motor_signal = 1;
        MTU2TGRD_4 = (long)(MOTOR_PWM_CYCLE - 1) * accele_r / 100;
    }
    else
    {
        // reverse
        Right_motor_signal = 0;
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
unsigned char dipsw_get(void) { return (dipsw.read() & 0x0f); }

//------------------------------------------------------------------//
// sensor Function
//------------------------------------------------------------------//
unsigned char sensor_inp(unsigned char mask)
{
    return (sensor_bin & mask); // 中
}

unsigned char sensor_inp2(unsigned char mask)
{
    return (sensor_bin2 & mask); // 中
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

    if ((getImage(114, 60) >= 230 || getImage(114, 62) >= 230 | getImage(114, 64) >= 230) && (getImage(43, 60) >= 230 || getImage(43, 62) >= 230 || getImage(43, 64) >= 230))
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
    if (b == 0x0f || b == 0x1f || b == 0x3f || b == 0x0e || b == 0x1e || b == 0x3e)
    {
        ret = 1;
    }
    // if (getImage(114, 60) >= 200 || getImage(114, 62) >= 200 | getImage(114, 64) >= 200 || getImage(114, 58) >= 200)
    // {
    //     ret = 1;
    // }
    // if (getImage(114, 60) >= 200 || getImage(114, 62) >= 200 | getImage(114, 64) >= 200 || getImage(114, 58) >= 200)
    // {
    //     ret = 1;
    // }
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
    if (b == 0xf0 || b == 0xf8 || b == 0xfc || b == 0x70 || b == 0x78 || b == 0x7c)
    {
        ret = 1;
    }
    // if (getImage(43, 60) >= 200 || getImage(43, 62) >= 200 || getImage(43, 64) >= 200 || getImage(43, 58) >= 200)
    // {
    //     ret = 1;
    // }
    // if (getImage(43, 60) >= 200 || getImage(43, 62) >= 200 || getImage(43, 64) >= 200 || getImage(43, 58) >= 200)
    // {
    //     ret = 1;
    // }
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

    // d[7] = getImage(20, gyou);

    // d[6] = getImage(23, gyou);

    // d[5] = getImage(26, gyou);

    // d[4] = getImage(29, gyou);

    // d[3] = getImage(120, gyou);

    // d[2] = getImage(123, gyou);

    // d[1] = getImage(126, gyou);

    // d[0] = getImage(129, gyou);

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

    //
    // min = max = data[0];
    // for (i = 30; i < 150; i++)
    // {
    //     if (max <= data[i])
    //     {
    //         max = d[i]; // 8個のうち、最大を見つける
    //         h_point = i;
    //     }
    //     if (min >= d[i])
    //     {
    //         min = d[i]; // 8個のうち、最小を見つける
    //         l_point = i;
    //     }
    // }
    // center = h_point - l_point;

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
    else if (sa_7_6 >= sa || sa_6_5 >= sa || sa_5_4 >= sa || sa_4_3 >= sa ||
             sa_3_2 >= sa || sa_2_1 >= sa || sa_1_0 >= sa)
    {
        // 隣同士の差が１つでも「差」以上なら、８点の（最大値－最小値）×0.7 +
        // 最小値　をしきい値とする
        shiki = (max - min) * 7 / 10 + min;
    }
    else
    {
        // 当てはまらなければ、しきい値を256とする、すなわちすべて0となる
        shiki = 256;
    }
    if (max < 100)
    {
        for (int i = 0; i < 8; i++)
        {
            d[i] = 0;
        }
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

unsigned char shikiichi_henkanline(int gyou, int s, int sa)
{
    int max, min, i, shiki;
    int d[8];
    int sa_7_6, sa_6_5, sa_5_4, sa_4_3, sa_3_2, sa_2_1, sa_1_0;
    unsigned char ret;

    // d[7] = getImage(31, gyou);

    // d[6] = getImage(43, gyou);

    // d[5] = getImage(54, gyou);

    // d[4] = getImage(71, gyou);

    // d[3] = getImage(88, gyou);

    // d[2] = getImage(105, gyou);

    // d[1] = getImage(116, gyou);

    // d[0] = getImage(128, gyou);

    d[7] = getImage(20, gyou);

    d[6] = getImage(23, gyou);

    d[5] = getImage(26, gyou);

    d[4] = getImage(29, gyou);

    d[3] = getImage(120, gyou);

    d[2] = getImage(123, gyou);

    d[1] = getImage(126, gyou);

    d[0] = getImage(129, gyou);

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

    //
    // min = max = data[0];
    // for (i = 30; i < 150; i++)
    // {
    //     if (max <= data[i])
    //     {
    //         max = d[i]; // 8個のうち、最大を見つける
    //         h_point = i;
    //     }
    //     if (min >= d[i])
    //     {
    //         min = d[i]; // 8個のうち、最小を見つける
    //         l_point = i;
    //     }
    // }
    // center = h_point - l_point;

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
    else if (sa_7_6 >= sa || sa_6_5 >= sa || sa_5_4 >= sa || sa_4_3 >= sa ||
             sa_3_2 >= sa || sa_2_1 >= sa || sa_1_0 >= sa)
    {
        // 隣同士の差が１つでも「差」以上なら、８点の（最大値－最小値）×0.7 +
        // 最小値　をしきい値とする
        shiki = (max - min) * 7 / 10 + min;
    }
    else
    {
        // 当てはまらなければ、しきい値を256とする、すなわちすべて0となる
        shiki = 256;
    }
    if (max < 100)
    {
        for (int i = 0; i < 8; i++)
        {
            d[i] = 0;
        }
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

unsigned char shikiichi_henkan2(int gyou, int s, int sa, int B1, int B2, int B3,
                                int B4, int B5, int B6, int B7, int B8)
{
    int max, min, i, shiki;
    int d[8];
    int sa_7_6, sa_6_5, sa_5_4, sa_4_3, sa_3_2, sa_2_1, sa_1_0;
    unsigned char ret;

    d[7] = getImage(B1, gyou);
    d[6] = getImage(B2, gyou);
    d[5] = getImage(B3, gyou);
    d[4] = getImage(B4, gyou);
    d[3] = getImage(B5, gyou);
    d[2] = getImage(B6, gyou);
    d[1] = getImage(B7, gyou);
    d[0] = getImage(B8, gyou);

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

    //
    // min = max = data[0];
    // for (i = 30; i < 150; i++)
    // {
    //     if (max <= data[i])
    //     {
    //         max = d[i]; // 8個のうち、最大を見つける
    //         h_point = i;
    //     }
    //     if (min >= d[i])
    //     {
    //         min = d[i]; // 8個のうち、最小を見つける
    //         l_point = i;
    //     }
    // }
    // center = h_point - l_point;

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
    else if (sa_7_6 >= sa || sa_6_5 >= sa || sa_5_4 >= sa || sa_4_3 >= sa ||
             sa_3_2 >= sa || sa_2_1 >= sa || sa_1_0 >= sa)
    {
        // 隣同士の差が１つでも「差」以上なら、８点の（最大値－最小値）×0.7 +
        // 最小値　をしきい値とする
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
void hennsa(void)
{
    // signed int hensa, hensa2;

    // static int atai[120][160];
    // for (int i = 0; i < 120; i++)
    // {
    //     for (int v = 0; v < 160; v++)
    //     {
    //         atai[i][v] = 0;
    //     }
    // }
    // static int gyoumax[120];

    // for (int i = 0; i < 120; i++)
    // {
    //     gyoumax[i] = 0;
    // }

    // static int sa[120][160];
    // for (int i = 0; i < 120; i++)
    // {
    //     for (int v = 0; v < 160; v++)
    //     {
    //         sa[i][v] = 0;
    //     }
    // }
    // static int bibunn[120][160];
    // for (int i = 0; i < 120; i++)
    // {
    //     for (int v = 0; v < 160; v++)
    //     {
    //         bibunn[i][v] = 0;
    //     }
    // }
    // static int max[120];
    // for (int i = 0; i < 120; i++)
    // {
    //     max[i] = 0;
    // }
    // static int ans[120];
    // for (int i = 0; i < 120; i++)
    // {
    //     ans[i] = 0;
    // }
    // static int gyou[120][160];
    // for (int i = 0; i < 120; i++)
    // {
    //     for (int v = 0; v < 160; v++)
    //     {
    //         gyou[i][v] = 0;
    //     }
    // }
    // static int countgyou[120];
    // for (int i = 0; i < 120; i++)
    // {
    //     countgyou[i] = 0;
    // }

    // static int ALLhennsamae[120];

    volatile signed int min, max120, max60, hensa, H, maxgyou, mingyou, sotosenngyou_R;

    volatile static signed int Sa = 0;
    volatile static signed int sotosenngyou_L2 = 40;
    volatile static signed int sotosenngyou_L = 40;
    volatile static signed int sotosenn_L = 255;
    volatile static signed int sotosenn_R = 255;
    volatile static signed int hensa2 = 0;
    volatile signed int count120 = 0;
    volatile signed int count60 = 0;
    volatile signed int unnti = 160;
    volatile signed int unnticount[320];
    for (int i = 0; i <= 319; i++)
    {
        unnticount[i] = 0;
    }
    volatile static bool saisyoflag = false;
    //  printf("aaaa\n");
    volatile signed int atai[320][320];
    for (int i = 0; i <= 319; i++)
    {
        for (int i2 = 0; i2 <= 319; i2++)
        {
            atai[i][i2] = 0;
        }
    }
    volatile signed int gyoumax[320];
    for (int i = 0; i <= 319; i++)
    {
        gyoumax[i] = 160;
    }
    volatile signed int gyoumaxsei[320];
    for (int i = 0; i <= 319; i++)
    {
        gyoumaxsei[i] = 160;
    }

    volatile signed int sa[320][320];
    for (int i = 0; i <= 319; i++)
    {
        for (int i2 = 0; i2 <= 319; i2++)
        {
            sa[i][i2] = 0;
        }
    }
    volatile signed int sasei[320][320];
    for (int i = 0; i <= 319; i++)
    {
        for (int i2 = 0; i2 <= 319; i2++)
        {
            sasei[i][i2] = 0;
        }
    }
    volatile signed int bibunn[320][320];
    for (int i = 0; i <= 319; i++)
    {
        for (int i2 = 0; i2 <= 319; i2++)
        {
            bibunn[i][i2] = 0;
        }
    }

    volatile signed int max[320];
    for (int i = 0; i <= 319; i++)
    {
        max[i] = 0;
    }

    volatile static signed int maxmae[320];
    if (saisyoflag == false)
    {
        for (int i = 0; i <= 319; i++)
        {
            maxmae[i] = 250;
            saisyoflag = true;
        }
    }

    volatile signed int ans[320];
    for (int i = 0; i <= 319; i++)
    {
        ans[i] = 0;
    }
    volatile signed int anssei[320];
    for (int i = 0; i <= 319; i++)
    {
        anssei[i] = 0;
    }

    volatile signed int gyou[320][320];
    for (int i = 0; i <= 319; i++)
    {
        for (int i2 = 0; i2 <= 319; i2++)
        {
            gyou[i][i2] = 0;
        }
    }

    volatile signed int gyousei[320][320];
    for (int i = 0; i <= 319; i++)
    {
        for (int i2 = 0; i2 <= 319; i2++)
        {
            gyousei[i][i2] = 0;
        }
    }
    volatile signed int countgyou[320];
    for (int i = 0; i <= 319; i++)
    {
        countgyou[i] = 0;
    }
    volatile signed int countgyousei[320];
    for (int i = 0; i <= 319; i++)
    {
        countgyousei[i] = 0;
    }

    volatile static signed int ALLhennsamae[320];
    volatile static signed int ALLhennsamae_sei[320];
    volatile static signed int ALLhennsamae_hu[320];

    min = 255;
    max120 = 0;
    max60 = 0;
    sotosenn_L = 255;
    sotosenn_R = 255;

    // あたいいれる

    for (int i = 0; i < 120; i++)
    {
        for (int i2 = 0; i2 < 160; i2++)
        {
            atai[i][i2] = getImage(i2, i);
        }
    }

    // さいだいみつける
    for (int i2 = 0; i2 < 160; i2++)
    {

        maxmae[i2] = max[i2];
    }

    for (int i = 0; i < 120; i++)
    {
        for (int i2 = 0; i2 < 160; i2++)
        {
            if (max[i] < atai[i][i2])
            {
                max[i] = atai[i][i2];
            }
        }
    }

    // ０か２００か８わり切り捨て

    for (int i = 0; i < 120; i++)
    {
        for (int i2 = 0; i2 < 160; i2++)
        {
            signed int v;
            v = atai[i][i2];
            // if (max[i] < 220)
            // {
            //     encoder.clearkura();
            // }
            // if (maxmae[i] - max[i] > -60)
            // {
            if (v > max[i] * 8 / 10)
            {
                v = 255;
            }

            // }

            // else
            // {
            //     if (v > 210)
            //     {
            //         v = 0;
            //     }
            //     else
            //     {
            //         if (v > maxmae[i] * 7 / 10)
            //         {
            //             v = 200;
            //         }
            //         else
            //         {
            //             v = 0;
            //         }
            //     }
            // }

            atai[i][i2] = v;
        }
    }

    // びぶん

    for (int i = 0; i < 120; i++)
    {
        for (int i2 = 0; i2 < 159; i2++)
        {
            bibunn[i][i2] = (atai[i][i2] - atai[i][i2 + 1]);
            bibunnG[i][i2] = (atai[i][i2] - atai[i][i2 + 1]);
        }
    }
    for (int i = 0; i < 120; i++)
    {
        // for (int i2 = 0; i2 < 159; i2++)
        // {
        //     bibunnG[i][i2] = bibunn[i][i2];
        // }
    }

    // さいしょうち出すーーー

    for (int i = 0; i < 119; i++)
    {
        for (int i2 = 0; i2 < 159; i2++)
        {
            if (bibunn[i][i2] < -7)
            {
                gyou[i][countgyou[i]] = i2;
                countgyou[i]++;
            }
        }
    }
    for (int i = 0; i < 119; i++)
    {
        for (int i2 = 0; i2 < 159; i2++)
        {

            if (bibunn[i][i2] > 7)
            {
                gyousei[i][countgyousei[i]] = i2;
                countgyousei[i]++;
            }
            else
            {
            }
        }
    }

    for (int i = 0; i < 120; i++)
    {
        if (countgyou[i] == 0)
        {
            gyou[i][countgyou[i]] = 60;

            countgyou[i]++;
        }
    }
    for (int i = 0; i < 120; i++)
    {

        if (countgyousei[i] == 0)
        {
            gyousei[i][countgyousei[i]] = 90;

            countgyousei[i]++;
        }
    }

    // 手前との差をだす

    ans[119] = 0;
    anssei[119] = 0;

    for (int i = 118; i >= 0; i--)
    {
        for (int i2 = 0; i2 <= countgyou[i]; i2++)
        {
            sa[i][i2] = abs(gyou[i][i2] - gyou[i + 1][ans[i + 1]]);
        }
        for (int i3 = 0; i3 <= countgyou[i]; i3++)
        {
            if (sa[i][i3] < gyoumax[i])
            {
                gyoumax[i] = sa[i][i3];
                ans[i] = i3;
            }
        }
    }
    for (int i = 118; i >= 0; i--)
    {

        for (int i2 = 0; i2 <= countgyousei[i]; i2++)
        {
            sasei[i][i2] = abs(gyousei[i][i2] - gyousei[i + 1][anssei[i + 1]]);
        }
        for (int i3 = 0; i3 <= countgyousei[i]; i3++)
        {
            if (sasei[i][i3] < gyoumaxsei[i])
            {
                gyoumaxsei[i] = sasei[i][i3];
                anssei[i] = i3;
            }
        }
    }

    for (int i = 100; i >= 0; i--)
    {

        if (abs(gyousei[i][anssei[i]] - gyousei[i - 1][anssei[i - 1]]) > 15)
        {
            gyousei[i - 1][anssei[i - 1]] = gyousei[i][anssei[i]];
        }
    }
    for (int i = 100; i >= 0; i--)
    {

        if (abs(gyou[i][ans[i]] - gyou[i - 1][ans[i - 1]]) > 15)
        {
            gyou[i - 1][ans[i - 1]] = gyou[i][ans[i]];
        }
    }
    // 比べて偏差出す

    hensa = 80 - gyou[60][ans[60]];
    for (int i = 0; i < 120; i++)
    {
        // if (abs((80-gyousei[i][anssei[i]]) - (80-ALLhennsamae_sei[i])) > 30 /*&& encoder.getTotalCount() >= 200*/)
        // {
        //     ALL_hennsa_sei[i] = ALLhennsamae_sei[i];
        // }
        // else {
        ALL_hennsa_sei[i] = 80 - gyousei[i][anssei[i]];
        ALL_hennsa_hu[i] = 80 - gyou[i][ans[i]];
        if (abs((80 - (gyou[i][ans[i]] + gyousei[i][anssei[i]]) / 2) - ALLhennsamae[i]) > 30 /*&& encoder.getTotalCount() >= 200*/)
        {
            ALL_hennsa[i] = ALLhennsamae[i];
            // ALL_hennsa[i] = 0;

            //  encoder.setvalue(-1000);
        }
        else
        {
            ALL_hennsa[i] = 80 - (gyou[i][ans[i]] + gyousei[i][anssei[i]]) / 2;
        }
        //}
        if(hazurereset==1){
            ALL_hennsa_sei[i] = 0;
            ALL_hennsa_hu[i] = 0;
            ALL_hennsa[i] = 0;
        }

        // if (abs((80 - gyou[i][ans[i]]) - ALLhennsamae_hu[i]) > 30 /*&& encoder.getTotalCount() >= 200*/)
        // {
        //     ALL_hennsa_hu[i] = ALLhennsamae_hu[i];
        // }
        // else {
        //}

      

        // else
        // {

        //}
    }
    hazurereset=0;

    for (int i = 0; i < 120; i++)
    {
        ALLhennsamae_sei[i] = ALL_hennsa_sei[i];
        ALLhennsamae_hu[i] = ALL_hennsa_hu[i];
        ALLhennsamae[i] = ALL_hennsa[i];
    }
    hensaG = hensa;

    if (abs(hensa - hensa2) > 40 && encoder.getTotalCount() >= 40000)
    {
        hensa = hensa2;
    }

    hensa2 = hensa;

    // return hensa;
}

int senn(int linegyou)
{
    volatile int leftdata[5][160];
    volatile int rightdata[5][160];
    volatile int sentor[5][160];
    volatile int alldata[5][160];
    volatile int bardata[5][160];

    volatile int assyukuL[160];
    volatile int assyukuR[160];
    volatile int assyukuS[160];
    volatile int assyukuA[160];
    volatile int assyukuB[160];

    volatile int lcount = 0;
    volatile int rcount = 0;
    volatile int scount = 0;
    volatile int bcount = 0;

    volatile int lmax = 0;
    volatile int rmax = 0;
    volatile int smax = 0;
    volatile int amax = 0;
    volatile int bmax = 0;

    volatile int noloine = 60;

    for (int i = 52; i <= 52 + 10; i++)
    {

        for (int j = 50; j <= 110; j++)
        {

            bardata[i][j] = getImage(j, i);
        }
    }

    for (int i = linegyou; i <= linegyou + 5; i++)
    {

        for (int j = 0; j <= 80; j++)
        {

            leftdata[i][j] = getImage(j, i);
            // ログ取得用(生データ)　データ
            // log_data[log_no].imageData[i- linegyou][j] = getImage(j, i);
        }
    }

    for (int i = linegyou; i <= linegyou + 3; i++)
    {

        for (int j = 0; j < 160; j++)
        {

            alldata[i][j] = getImage(j, i);
            // ログ取得用(生データ)　データ
            // log_data[log_no].imageData[i- linegyou][j] = getImage(j, i);
        }
    }

    for (int i = linegyou; i <= linegyou + 5; i++)
    {

        for (int j = 81; j <= 159; j++)
        {

            rightdata[i][j] = getImage(j, i);

            // ログ取得用(生データ)　データ
            // log_data[log_no].imageData[i- linegyou][j] = getImage(j, i);
        }
    }

    for (int i = noloine; i <= noloine + 3; i++)
    {

        for (int j = 50; j <= 110; j++)
        {

            sentor[i][j] = getImage(j, i);
        }
    }

    for (int i = linegyou; i <= linegyou + 5; i++)
    {

        for (int j = 0; j <= 80; j++)
        {
            if (leftdata[i][j] > assyukuL[j])
            {
                assyukuL[j] = leftdata[i][j];
            }
            if (leftdata[i][j] > lmax)
            {
                lmax = leftdata[i][j];
            }
        }
    }

    for (int i = linegyou; i <= linegyou + 5; i++)
    {

        for (int j = 81; j <= 159; j++)
        {
            if (rightdata[i][j] > assyukuR[j])
            {
                assyukuR[j] = rightdata[i][j];
            }
            if (rightdata[i][j] > rmax)
            {
                rmax = rightdata[i][j];
            }
        }
    }
    for (int i = 52; i <= 52 + 10; i++)
    {

        for (int j = 50; j <= 110; j++)
        {
            if (bardata[i][j] > assyukuB[j])
            {
                assyukuB[j] = bardata[i][j];
            }
            if (bardata[i][j] > bmax)
            {
                bmax = bardata[i][j];
            }
        }
    }
    for (int i = linegyou; i <= linegyou + 3; i++)
    {

        for (int j = 0; j <= 159; j++)
        {
            if (alldata[i][j] > log_data[log_no].imageData0[j])
            {
                log_data[log_no].imageData0[j] = alldata[i][j];
            }
            if (alldata[i][j] > amax)
            {
                amax = alldata[i][j];
            }
        }
    }

    for (int i = noloine; i <= noloine + 3; i++)
    {

        for (int j = 50; j <= 110; j++)
        {
            if (sentor[i][j] > assyukuS[j])
            {
                assyukuS[j] = sentor[i][j];
            }
            if (sentor[i][j] > smax)
            {
                smax = sentor[i][j];
            }
        }
    }

    for (int j = 10; j <= 80; j++)
    {
        if (assyukuL[j] > lmax * 0.75)
        {
            lcount++;
        }
    }
    for (int j = 81; j <= 159; j++)
    {
        if (assyukuR[j] > rmax * 0.75)
        {
            rcount++;
        }
    }

    for (int j = 50; j <= 110; j++)
    {
        if (assyukuS[j] > 200)
        {
            scount++;
        }
    }

    for (int j = 50; j <= 110; j++)
    {
        if (assyukuB[j] > bmax * 0.7)
        {
            bcount++;
        }
    }

    if (lcount > 60 && lmax >= 180)
    {
        leftflag = 1;
    }

    else
    {
        leftflag = 0;
    }

    if (rcount > 60 && rmax >= 180)
    {
        rightflag = 1;
    }

    else
    {
        rightflag = 0;
    }

    if (lcount > 60 && rcount > 60 && rmax >= 180 && lmax >= 180)
    {
        crossflag = 1;
    }

    else
    {
        crossflag = 0;
    }

    if (scount <= 3)
    {
        nolineflag = 1;
    }

    else
    {
        nolineflag = 0;
    }

    if (bcount > 40)
    {
        barflag = 1;
    }
    else
    {
        barflag = 0;
    }
    return 0;
}
///********************************************************************
// イメージ領域のx,yの値(0-255)を取得
// 引数：x列数(0-159) , y行数(0-119)
// 戻り値：0～255
///********************************************************************
int getImage(int ix, int iy) { return ImageData_B[ix + 160U * iy]; }

//------------------------------------------------------------------//
// End of file
//------------------------------------------------------------------//