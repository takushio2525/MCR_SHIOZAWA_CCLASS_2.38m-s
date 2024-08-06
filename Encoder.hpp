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
private:
    long long _total_cnt;
    long long _cnt;
    long long _cnt_kyori;
    long long _cnt_kyorikousinn;
    long long _cnt_kyorikousinnmae;
    long long _course_cnt;

    long long _cnt1;
    long long _cntmaga = 0;

    long long cntave;
    long long cntgoukei;
    long long cntdata[4];

    float _rc;
    float _avg;
    float _a;
    int _filter_value[Filter_N];
    int _filter_cnt;

public:
    Encoder();
    void init(void);        // 初期化
    void update(void);      // カウント更新
    void kyoriupdate(void); // カウント更新

    void clear(void);     // 全カウントクリア
    void clearkura(void); // 全カウントクリア

    int getTotalCount(void); // トータルカウント取得
    int getMagaCount(void); // クランクカウント取得
    int getCourseCount(void);
    void setmaga(int value);

    int getCnt(void);           // 10ms間のカウントを取得
    float getFilteredCnt(void); // RCフィルターを通した値を取得
    void setvalue(int value);
};
