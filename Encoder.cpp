/*
 * Encoder.cpp
 * Created on: 2022 10/3
 * Author: Mishima
 */

#include "Encoder.hpp"

Encoder::Encoder()
{
    _total_cnt = 0;
    _cnt = 0;
    _cnt1 = 0;
    _a = 0.5;
    _filter_cnt = 0;
    for (int i = 0; i < Filter_N; i++)
    {
        _filter_value[i] = 0;
    }
}

//------------------------------------------------------------------//
// Initialize MTU2 Phase Count functions
//------------------------------------------------------------------//
// MTU2_1
// Phase counting mode 2
// TCLKA(P1_0)  :Enconder A
// TCLKB(P1_10) :Enconder B
//------------------------------------------------------------------//
void Encoder::init(void)
{
    CPGSTBCR3 &= ~0x08;

    GPIOPIBC1 &= ~0x0401;
    GPIOPBDC1 &= ~0x0401;
    GPIOPM1 |= 0x0401;
    GPIOPMC1 &= ~0x0401;
    GPIOPIPC1 &= ~0x0401;

    GPIOPBDC1 &= ~0x0401;

    GPIOPFC1 |= 0x0400;
    GPIOPFCE1 |= 0x0401;

    GPIOPIPC1 |= 0x0401;
    GPIOPMC1 |= 0x0401;

    MTU2TSTR &= ~0x02; // カウンタストップ
    MTU2TCR_1 = 0x14;  // 上　両エッジ　下　外部カウンタ

    MTU2TMDR_1 |= 0x00; // 位相計数モード2 https://www.renesas.com/jp/ja/document/mah/rza1h-group-rza1m-group-users-manual-hardware 463ページ
                        // MTU2TMDR_3|=0x00;//通常モード
    MTU2TCNT_1 = 0x00;  // 32767
    MTU2TIOR_1 |= 0x0A; // 両エッジ設定
    MTU2TSTR |= 0x02;   // タイマースタート

    // MTU2TCR_3=0x10;//1x：両エッジでカウント
    // MTU2TIORH_3 |= 0xAA;//両方カウント設定
    // MTU2TCNT_3 = 0x7FFF;//ぜろくり？？
}

// カウント更新
void Encoder::update(void)
{
    float sum = 0;
    // _cnt = 0 - (MTU2TCNT_1 - 32767);
    // MTU2TCNT_1 = 0x7FFF;
    // _cnt =MTU2TCNT_1;
    //  MTU2TCNT_1 = 0x00;

    _cnt = MTU2TCNT_1;
    MTU2TCNT_1 = 0x0000;
    _cnt_kyorikousinnmae = 0;
    _total_cnt = _total_cnt + _cnt;
    _cntmaga = _cntmaga + _cnt;

    _course_cnt = _course_cnt + _cnt;

    _cnt1 = _cnt1 + _cnt;
    _rc = _a * _rc + (1 - _a) * (float)_cnt;

    _filter_value[_filter_cnt] = _cnt;
    _filter_cnt++;
    if (_filter_cnt > Filter_N)
    {
        _filter_cnt = 0;
    }
    for (int i = 0; i < Filter_N; i++)
    {
        sum += _filter_value[i];
    }
    _avg = sum / Filter_N;

    cntdata[3] = cntdata[2];
    cntdata[2] = cntdata[1];
    cntdata[1] = cntdata[0];
    cntdata[0] = _cnt;
    cntgoukei = cntdata[0] + cntdata[1] + cntdata[2] + cntdata[3];
    cntave = cntgoukei / 4;
}

void Encoder::kyoriupdate()
{
    // _cnt_kyorikousinn=MTU2TCNT_1-_cnt_kyorikousinnmae;
    // _total_cnt = _total_cnt + _cnt_kyorikousinn;
    // _cntclank = _cntclank + _cnt_kyorikousinn;
    // _cnt_kyorikousinnmae=_cnt_kyorikousinn;
}

// 全カウントクリア
void Encoder::clear(void)
{
    _total_cnt = 0;

    //_cntclank = 0;
    // _cnt = 0;
    // _cnt1 = 0;
    // MTU2TCNT_3 = 0x00;
}

// void Encoder::clearkura(void)
// {
//     //_total_cnt = 0;

//     _cntclank = 0;
//     // _cnt = 0;
//     // _cnt1 = 0;
//     // MTU2TCNT_3 = 0x00;
// }

void Encoder::setvalue(int value)
{
    _total_cnt = value;
    // _cnt = 0;
    // _cnt1 = 0;
    // MTU2TCNT_3 = 0x00;
}

void Encoder::setmaga(int value)
{
    _cntmaga = value;
    // _cnt = 0;
    // _cnt1 = 0;
    // MTU2TCNT_3 = 0x00;
}

// トータルカウント取得
int Encoder::getTotalCount(void)
{
    return _total_cnt;
}

int Encoder::getCourseCount(void)
{
    return _course_cnt;
}

// 10ms間のカウントを取得
int Encoder::getCnt(void)
{
    // return _cnt;
    return cntave;
}

int Encoder::getMagaCount(void)
{
    return _cntmaga;
}
// フィルターを通した値を取得
float Encoder::getFilteredCnt(void)
{
    return _avg;
}
