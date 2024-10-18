/*
 * Encoder.hpp
 * Created on: 2022 10/3
 * Author: Mishima
 */

#pragma once

#include <algorithm>

#define Filter_N 2

#include "iodefine.h"
class Encoder
{
public:
    volatile int _total_cnt;
    volatile long long _cnt;
    volatile long long _cnt_kyori;
    volatile long long _cnt_kyorikousinn;
    volatile long long _cnt_kyorikousinnmae;
    volatile long long _course_cnt;

    volatile long long _cnt1;
    volatile long long _cntmaga = 0;

    volatile long long cntave;
    volatile long long cntgoukei;
    volatile long long cntdata[4];

    volatile float _rc;
    volatile float _avg;
    volatile float _a;
    volatile int _filter_value[Filter_N];
    volatile int _filter_cnt;

public:
    Encoder();
    void init(void);        // 初期化
    void update(void);      // カウント更新
    void kyoriupdate(void); // カウント更新

    void clear(void);     // 全カウントクリア
    void clearkura(void); // 全カウントクリア

    int getTotalCount(void); // トータルカウント取得
    int getMagaCount(void);  // クランクカウント取得
    int getCourseCount(void);
    void setmaga(int value);

    int getCnt(void);           // 10ms間のカウントを取得
    float getFilteredCnt(void); // RCフィルターを通した値を取得
    void setvalue(int value);
};
