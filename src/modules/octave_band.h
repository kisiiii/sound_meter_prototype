#ifndef OCTAVE_BAND_H
#define OCTAVE_BAND_H

#include "../common/common.h"

// Bボタン機能：オクターブバンド分析・表示・CSV保存

// Leq計算用のバッファ（1秒間のデータを保存）
extern float leqBuffer[6][29];  // 29個のサンプル（1秒分：1024/30000*29≈1秒）
extern int leqBufferIndex;
extern bool leqBufferFull;

// B画面の初期化フラグ
extern bool octaveBandScreenInitialized;
extern unsigned long lastLeqUpdateTime;  // 1秒ごと更新用タイマー

// 1秒間Leq計算関数
void calculateLeq(OctaveBandData_t* result);

// LAeq合算計算関数
void calculateLAeq();

// 1/1オクターブバンド分析関数（arduinoFFTのマグニチュード結果用）
void analyzeOctaveBands(float* magnitudeData, int fftSize, OctaveBandData_t* result);

// オクターブバンド画面関数
void initOctaveBandScreen();
void updateOctaveBandBars();

// 瞬時ピーク情報表示
void showInstantPeakInfo();

#endif // OCTAVE_BAND_H