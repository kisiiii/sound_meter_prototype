#ifndef COMMON_H
#define COMMON_H

#include <M5Stack.h>
#include <driver/i2s.h>
#include "arduinoFFT.h"
#include <math.h>
#include <SD.h>
#include <time.h>

// ピン定義
#define PIN_CLK  22
#define PIN_DATA 21

// モード定義
#define MODE_MIC 0

// 音声処理設定
#define SAMPLE_RATE 30000  // 統一されたサンプリング周波数
#define FFT_SIZE 1024      // FFTサイズ
#define NYQUIST_FREQ (SAMPLE_RATE / 2)  // ナイキスト周波数

// マイクの仕様
#define MIC_SENSITIVITY_DBFS -22.0  // dBFS
#define MIC_REF_DB 94.0            // 1kHz, 94dB SPL時の基準

// CSV保存用
#define CSV_FILE "/logger.csv"
extern const unsigned long SAVE_INTERVAL_MS;  // 1秒間隔で保存
extern const char* freqs[];
extern bool csvLoggingEnabled;  // CSV自動保存の有効/無効フラグ

// 1/1オクターブバンドの中心周波数 (ISO 266準拠、8kHz以下)
extern const float OCTAVE_CENTER_FREQ[];
extern const int OCTAVE_BAND_COUNT;

// 周波数帯域別補正値 (dB)
extern const float CORRECTION_VALUES[];

// A特性補正値 (dB) - 125Hz, 250Hz, 500Hz, 1000Hz, 2000Hz, 4000Hz
extern const float A_WEIGHTING_VALUES[];

// グローバル変数
extern unsigned long lastSaveTime;
extern float leqResults[];
extern float laeqResult;  // LAeq合算値

// 表示モード
enum DisplayMode {
    MODE_REALTIME_FFT = 0,
    MODE_OCTAVE_BAND = 1,
    MODE_SHOW_PEAK_INFO = 2,
    MODE_SET_TIME = 3
};

extern DisplayMode currentDisplayMode;

// オクターブバンド解析用の構造体
typedef struct {
    float bandLevels[6];     // 各バンドのdB値
    float bandLeq[6];        // 各バンドの1秒間Leq値
    float peakFrequency;
    float peakDB;
    int peakBandIndex;       // ピークのバンドインデックス
    bool newDataAvailable;
} OctaveBandData_t;

extern OctaveBandData_t lastOctaveBandData;
extern OctaveBandData_t instantOctaveBandData;  // 瞬時値保存用

// I2Sメッセージ構造体
typedef struct {
    uint8_t state;
    void* audioPtr;
    uint32_t audioSize;
} i2sQueueMsg_t;

// スプライト・キュー
extern TFT_eSprite DisFFTbuff;
extern TFT_eSprite OctaveBandBuff;
extern QueueHandle_t fftvalueQueue;
extern QueueHandle_t i2sstateQueue;
extern QueueHandle_t octaveBandQueue;

// 共通関数プロトタイプ
void header(const char *string, uint16_t color);
bool InitI2SSpakerOrMic(int mode);
float calculateDB(float amplitude);
void initCSVFileIfNeeded();
void saveLeqToCSV(float leqValues[6]);

#endif // COMMON_H