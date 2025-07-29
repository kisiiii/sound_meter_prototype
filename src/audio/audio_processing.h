#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H

#include "../common/common.h"

// 音声処理・FFT解析モジュール

// arduinoFFT関連
extern ArduinoFFT<float> FFT;
extern float vReal[FFT_SIZE];
extern float vImag[FFT_SIZE];
extern bool captureInstantPeak;  // 瞬時値取得フラグ

// マイクロフォン・FFT処理関数
void microPhoneSetup();
static void i2sMicroFFTtask(void *arg);

#endif // AUDIO_PROCESSING_H