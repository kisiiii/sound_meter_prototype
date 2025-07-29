//Remove the M5GO base when using this example, otherwise it will not work properly,PDM Unit connected PORT-A

#include <M5Stack.h>
#include <driver/i2s.h>
#include "arduinoFFT.h"
#include <math.h>
#include <SD.h>
#include <time.h>

#define PIN_CLK  22
#define PIN_DATA 21

#define MODE_MIC 0
#define SAMPLE_RATE 30000  // 統一されたサンプリング周波数
#define FFT_SIZE 1024      // FFTサイズ（i2sMicroFFTtask内でも1024で使われている）
#define NYQUIST_FREQ (SAMPLE_RATE / 2)  // ナイキスト周波数

// マイクの仕様
#define MIC_SENSITIVITY_DBFS -22.0  // dBFS
#define MIC_REF_DB 94.0            // 1kHz, 94dB SPL時の基準

// CSV保存用
#define CSV_FILE "/logger.csv"
unsigned long lastSaveTime = 0;
const unsigned long SAVE_INTERVAL_MS = 1000;  // 1秒間隔で保存（1000ms = 1秒）
const char* freqs[] = { "63", "125", "250", "500", "1000", "2000", "4000" };
bool csvLoggingEnabled = false;  // CSV自動保存の有効/無効フラグ

// Leq計算結果格納用
float leqResults[7] = {0};

// 1/1オクターブバンドの中心周波数 (ISO 266準拠、8kHz以下)
const float OCTAVE_CENTER_FREQ[] = {63, 125, 250, 500, 1000, 2000, 4000};
const int OCTAVE_BAND_COUNT = 7;

TFT_eSprite DisFFTbuff =  TFT_eSprite(&M5.Lcd);
TFT_eSprite OctaveBandBuff = TFT_eSprite(&M5.Lcd);  // オクターブバンド表示用
static QueueHandle_t fftvalueQueue = nullptr;
static QueueHandle_t i2sstateQueue = nullptr;
static QueueHandle_t octaveBandQueue = nullptr;  // オクターブバンドデータ用キュー

// オクターブバンド解析用の構造体
typedef struct {
    float bandLevels[OCTAVE_BAND_COUNT];  // 各バンドのdB値
    float bandLeq[OCTAVE_BAND_COUNT];     // 各バンドの1秒間Leq値
    float peakFrequency;
    float peakDB;
    int peakBandIndex;  // ピークのバンドインデックス
    bool newDataAvailable;
} OctaveBandData_t;

// 表示モード
enum DisplayMode {
    MODE_REALTIME_FFT = 0,
    MODE_OCTAVE_BAND = 1,
    MODE_SHOW_PEAK_INFO = 2,
    MODE_SET_TIME = 3
};

// arduinoFFT関連
ArduinoFFT<float> FFT = ArduinoFFT<float>();
static float vReal[FFT_SIZE];
static float vImag[FFT_SIZE];
static bool captureInstantPeak = false;  // 瞬時値取得フラグ

// Leq計算用のバッファ（1秒間のデータを保存）
static float leqBuffer[OCTAVE_BAND_COUNT][29];  // 29個のサンプル（1秒分：1024/30000*29≈1秒）
static int leqBufferIndex = 0;
static bool leqBufferFull = false;

// B画面の初期化フラグ
static bool octaveBandScreenInitialized = false;
static unsigned long lastLeqUpdateTime = 0;  // 1秒ごと更新用タイマー

DisplayMode currentDisplayMode = MODE_REALTIME_FFT;
OctaveBandData_t lastOctaveBandData;
OctaveBandData_t instantOctaveBandData;  // 瞬時値保存用

void header(const char *string, uint16_t color)
{
    M5.Lcd.fillScreen(color);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.fillRect(0, 0, 320, 30, BLACK);
    M5.Lcd.setTextDatum(TC_DATUM);
    M5.Lcd.drawString(string, 160, 3, 4); 
}

typedef struct
{
    uint8_t state;
    void* audioPtr;
    uint32_t audioSize;
}i2sQueueMsg_t;

bool InitI2SSpakerOrMic(int mode)
{
    Serial.println("Initializing I2S...");
    
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 2,
        .dma_buf_len = 128,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };
    
    if (mode == MODE_MIC)
    {
        i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM);
    }

    esp_err_t err = i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    if (err != ESP_OK) {
        Serial.printf("I2S driver install failed: %s\n", esp_err_to_name(err));
        return false;
    }
    
    i2s_pin_config_t pin_config;
    pin_config.bck_io_num = I2S_PIN_NO_CHANGE;
    pin_config.ws_io_num = PIN_CLK;
    pin_config.data_out_num = I2S_PIN_NO_CHANGE;
    pin_config.data_in_num = PIN_DATA;
    
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 4, 0)
    pin_config.mck_io_num = I2S_PIN_NO_CHANGE;
#endif

    err = i2s_set_pin(I2S_NUM_0, &pin_config);
    if (err != ESP_OK) {
        Serial.printf("I2S set pin failed: %s\n", esp_err_to_name(err));
        return false;
    }
    
    err = i2s_set_clk(I2S_NUM_0, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
    if (err != ESP_OK) {
        Serial.printf("I2S set clock failed: %s\n", esp_err_to_name(err));
        return false;
    }
    
    Serial.println("I2S initialized successfully");
    return true;
}

// dB計算関数
float calculateDB(float amplitude) {
    if (amplitude <= 0) return -100.0;  // 極小値処理
    float dbfs = 20.0 * log10(amplitude / 32768.0);
    float db_spl = dbfs - MIC_SENSITIVITY_DBFS + MIC_REF_DB;
    return db_spl;
}

// 1秒間Leq計算関数
void calculateLeq(OctaveBandData_t* result) {
    for (int band = 0; band < OCTAVE_BAND_COUNT; band++) {
        // 現在のdB値をリニア値に変換してバッファに保存
        float linearValue = pow(10.0, result->bandLevels[band] / 10.0);
        leqBuffer[band][leqBufferIndex] = linearValue;
        
        // 1秒間のLeq計算
        if (leqBufferFull || leqBufferIndex > 0) {
            float sum = 0.0;
            int count = leqBufferFull ? 29 : (leqBufferIndex + 1);  // 29サンプルで1秒
            
            for (int i = 0; i < count; i++) {
                sum += leqBuffer[band][i];
            }
            
            float avgLinear = sum / count;
            result->bandLeq[band] = 10.0 * log10(avgLinear);
        } else {
            result->bandLeq[band] = result->bandLevels[band];  // 初期値
        }
        
        // グローバルのleqResults配列を更新
        leqResults[band] = result->bandLeq[band];
    }
    
    // バッファインデックス更新
    leqBufferIndex++;
    if (leqBufferIndex >= 29) {  // 29サンプルで1秒
        leqBufferIndex = 0;
        leqBufferFull = true;
    }
}

// 1/1オクターブバンド分析関数（arduinoFFTのマグニチュード結果用）
void analyzeOctaveBands(float* magnitudeData, int fftSize, OctaveBandData_t* result) {
    float frequencyResolution = (float)SAMPLE_RATE / fftSize;
    
    // 各バンドの初期化
    for (int band = 0; band < OCTAVE_BAND_COUNT; band++) {
        result->bandLevels[band] = -100.0;  // 初期値
        result->bandLeq[band] = -100.0;     // 初期値
    }
    
    // 各オクターブバンドの上限・下限周波数を計算 (1/1オクターブ)
    for (int band = 0; band < OCTAVE_BAND_COUNT; band++) {
        float centerFreq = OCTAVE_CENTER_FREQ[band];
        float lowerFreq = centerFreq / sqrt(2.0);  // fc/√2
        float upperFreq = centerFreq * sqrt(2.0);  // fc×√2
        
        // FFTビンのインデックス範囲を計算
        int lowerBin = (int)(lowerFreq / frequencyResolution);
        int upperBin = (int)(upperFreq / frequencyResolution);
        
        // 範囲チェック
        if (lowerBin < 1) lowerBin = 1;  // DC成分をスキップ
        if (upperBin >= fftSize / 2) upperBin = fftSize / 2 - 1;
        
        // バンド内のパワーを積算
        float bandPower = 0.0;
        int binCount = 0;
        
        for (int bin = lowerBin; bin <= upperBin; bin++) {
            // arduinoFFTのマグニチュード結果を直接使用
            float amplitude = magnitudeData[bin];
            bandPower += amplitude * amplitude;  // パワーとして積算
            binCount++;
        }
        
        if (binCount > 0) {
            // RMS値を計算してdBに変換
            float rmsValue = sqrt(bandPower / binCount);
            result->bandLevels[band] = calculateDB(rmsValue);
        }
    }
    
    // 1秒間Leq計算
    calculateLeq(result);
    
    // ピークバンドを特定（Leq値から）
    result->peakDB = -100.0;
    result->peakBandIndex = 0;
    
    for (int band = 0; band < OCTAVE_BAND_COUNT; band++) {
        if (result->bandLeq[band] > result->peakDB) {
            result->peakDB = result->bandLeq[band];
            result->peakBandIndex = band;
            result->peakFrequency = OCTAVE_CENTER_FREQ[band];
        }
    }
    
    result->newDataAvailable = true;
    
    // デバッグ出力
    static int debug_count = 0;
    if (debug_count++ % 29 == 0) {  // 約1秒に1回出力（29サンプルに1回）
        Serial.printf("Peak Leq Band: %.0f Hz, %.1f dB SPL\n", 
                      result->peakFrequency, result->peakDB);
    }
}

static void i2sMicroFFTtask(void *arg)
{
    uint8_t FFTDataBuff[128];
    uint8_t FFTValueBuff[24];
    uint8_t* microRawData = (uint8_t*)calloc(2048,sizeof(uint8_t));
    size_t bytesread;
    int16_t* buffptr;
    double data = 0;
    float adc_data;
    uint16_t ydata;
    uint32_t subData;

    uint8_t state = MODE_MIC;
    i2sQueueMsg_t QueueMsg;
    OctaveBandData_t octaveBandData;
    
    int no_data_count = 0;
    
    while(1)
    {
        if( xQueueReceive(i2sstateQueue,&QueueMsg,(TickType_t)0) == pdTRUE)
        {
            if( QueueMsg.state == MODE_MIC )
            {
                InitI2SSpakerOrMic(MODE_MIC);
                state = MODE_MIC;
            }
        }
        else if( state == MODE_MIC )
        {
            esp_err_t result = i2s_read(I2S_NUM_0, (char *)microRawData, 2048, &bytesread, (100 / portTICK_RATE_MS));
            
            if (result != ESP_OK) {
                Serial.printf("I2S read error: %s\n", esp_err_to_name(result));
            }
            
            if (bytesread == 0) {
                no_data_count++;
                if (no_data_count % 100 == 0) {
                    Serial.printf("No data received (count: %d)\n", no_data_count);
                }
                continue;
            } else {
                if (no_data_count > 0) {
                    Serial.printf("Data received: %d bytes\n", bytesread);
                    no_data_count = 0;
                }
            }
            
            buffptr = (int16_t*)microRawData;

            // デバッグ用サンプル範囲確認
            static int debug_counter = 0;
            if (debug_counter++ % 100 == 0) {
                int16_t min_val = INT16_MAX, max_val = INT16_MIN;
                for (int i = 0; i < 10; i++) {
                    if (buffptr[i] < min_val) min_val = buffptr[i];
                    if (buffptr[i] > max_val) max_val = buffptr[i];
                }
                Serial.printf("Sample range: %d to %d\n", min_val, max_val);
            }

            // vRealとvImagの初期化
            for (int i = 0; i < FFT_SIZE; i++) {
                vImag[i] = 0.0;
            }
            
            for ( int count_n = 0; count_n < FFT_SIZE; count_n++)
            {
                adc_data = (float)map(buffptr[count_n], INT16_MIN, INT16_MAX, -2000, 2000);
                vReal[count_n] = adc_data;  // 窓関数はarduinoFFTライブラリで適用
            }
            
            // arduinoFFTを実行
            FFT.windowing(vReal, FFT_SIZE, FFT_WIN_TYP_HANN, FFT_FORWARD);
            FFT.compute(vReal, vImag, FFT_SIZE, FFT_FORWARD);
            FFT.complexToMagnitude(vReal, vImag, FFT_SIZE);

            // FFTの結果をvRealに保存（マグニチュード計算済み）

            // リアルタイム表示用のデータ処理（8kHzまでの範囲）
            for ( int count_n = 1; count_n < FFT_SIZE / 4; count_n++)
            {
                data = vReal[count_n];  // 既にマグニチュード計算済み
                if ((count_n - 1) < 128)
                {
                    data = ( data > 2000 ) ? 2000 : data;
                    ydata = map(data, 0, 2000, 0, 255);
                    FFTDataBuff[128 - count_n] = ydata;
                }
            }

            for( int count = 0; count < 24; count++ )
            {
                subData = 0;
                for( int count_i = 0; count_i < 5; count_i++ )
                {
                    subData += FFTDataBuff[count * 5 + count_i ];
                }
                subData /= 5;
                FFTValueBuff[count] = map(subData,0,255,0,8);
            }
            
            // オクターブバンド解析（vRealを使用）
            analyzeOctaveBands(vReal, FFT_SIZE, &octaveBandData);
            
            // 瞬時値取得が要求された場合
            if (captureInstantPeak) {
                instantOctaveBandData = octaveBandData;  // 瞬時値を保存
                captureInstantPeak = false;
                Serial.printf("Instant Peak Captured: %.0f Hz, %.1f dB SPL\n", 
                             instantOctaveBandData.peakFrequency, instantOctaveBandData.peakDB);
            }
            
            xQueueOverwrite(octaveBandQueue, &octaveBandData);
            xQueueSend( fftvalueQueue, (void * )&FFTValueBuff, 0 );
        }
        else
        {
            delay(10);
        }
    }
}

void microPhoneSetup()
{
    fftvalueQueue = xQueueCreate(5, 24 * sizeof(uint8_t));
    if( fftvalueQueue == 0 )
    {
        Serial.println("Failed to create FFT queue");
        return;
    }

    i2sstateQueue = xQueueCreate(5, sizeof(i2sQueueMsg_t));
    if( i2sstateQueue == 0 )
    {
        Serial.println("Failed to create I2S state queue");
        return;
    }
    
    octaveBandQueue = xQueueCreate(1, sizeof(OctaveBandData_t));
    if( octaveBandQueue == 0 )
    {
        Serial.println("Failed to create octave band queue");
        return;
    }

    bool result = InitI2SSpakerOrMic(MODE_MIC);
    if (!result) {
        Serial.println("Failed to initialize I2S");
        return;
    }
    
    xTaskCreatePinnedToCore(i2sMicroFFTtask, "microPhoneTask", 8192, NULL, 3, NULL, 0);
    DisFFTbuff.createSprite(320,54);
    OctaveBandBuff.createSprite(320, 200);
    
    Serial.println("Microphone setup completed");
}

void MicroPhoneFFT()
{
    uint8_t FFTValueBuff[24];
    if (xQueueReceive( fftvalueQueue, (void * )&FFTValueBuff, 100 / portTICK_RATE_MS ) == pdTRUE)
    {
        DisFFTbuff.fillRect(0,0,320,54,DisFFTbuff.color565(0x00,0x00,0x00));
        uint32_t colorY = DisFFTbuff.color565(0xff,0x9c,0x00);
        uint32_t colorG = DisFFTbuff.color565(0x66,0xff,0x00);
        uint32_t colorRect;
        for( int x = 0; x < 24; x++ )
        {
            for( int y = 0; y < 9; y++ )
            {
                if( y < FFTValueBuff[23-x] )
                {
                    colorRect = colorY;
                }
                else if( y == FFTValueBuff[23-x] )
                {
                    colorRect = colorG;
                }
                else
                {
                    continue;
                }
                DisFFTbuff.fillRect(x*12,54-y*6 - 5,5,5,colorRect);
            }
        }
        
        DisFFTbuff.pushSprite(20, 120);
        
        M5.Lcd.setTextColor(WHITE, BLACK);
        M5.Lcd.setTextSize(1);
        M5.Lcd.setTextDatum(TC_DATUM);

        const int graphLeft = 20;
        const int graphWidth = 288;  // 24バンド × 12px
        
        // FFT実際の周波数に対応した目盛り
        // 24バー × 5FFTビン/バー = 120FFTビン使用 (0～3515Hz)
        const float freqResolution = (float)SAMPLE_RATE / 1024;  // 29.297Hz
        const float maxDisplayFreq = 120 * freqResolution;  // 3515Hz
        
        // 実際の表示範囲に合わせた目盛り
        const int frequencies[] = {0, 500, 1000, 1500, 2000, 2500, 3000, 3500};
        const int numFreqs = 8;
        
        for (int i = 0; i < numFreqs; i++) {
            int freq = frequencies[i];
            // リニアスケールでのx位置計算（実際の周波数範囲に対応）
            int x = graphLeft + (int)(graphWidth * freq / maxDisplayFreq);
            
            char label[8];
            if (freq == 0) {
                sprintf(label, "0");
            } else if (freq >= 1000) {
                sprintf(label, "%.1fk", freq / 1000.0);
            } else {
                sprintf(label, "%d", freq);
            }

            M5.Lcd.drawString(label, x, 178);
        }
    }
}

void initOctaveBandScreen()
{
    // 画面全体をクリア（一度だけ）
    M5.Lcd.fillScreen(BLACK);
    
    // タイトル
    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setTextDatum(TC_DATUM);
    M5.Lcd.drawString("1/1 Octave Band Leq (1sec)", 160, 20);
    
    // dBスケール描画（画面に直接描画）
    M5.Lcd.setTextColor(CYAN, BLACK);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setTextDatum(TL_DATUM);
    for (int db = 40; db <= 120; db += 20) {
        int y = 220 - ((db - 40) * 140 / 80);  // 40-120dBの範囲で140pixel
        char label[8];
        sprintf(label, "%d", db);
        M5.Lcd.drawString(label, 5, y - 4);
        M5.Lcd.drawLine(25, y, 310, y, DARKGREY);  // グリッド線
    }
    
    // 中心周波数ラベル（固定部分）
    int barWidth = 35;
    int barSpacing = 40;
    int startX = 30;
    
    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setTextDatum(TC_DATUM);
    
    for (int i = 0; i < OCTAVE_BAND_COUNT; i++) {
        int x = startX + i * barSpacing;
        char freqLabel[8];
        if (OCTAVE_CENTER_FREQ[i] >= 1000) {
            sprintf(freqLabel, "%.0fk", OCTAVE_CENTER_FREQ[i] / 1000);
        } else {
            sprintf(freqLabel, "%.0f", OCTAVE_CENTER_FREQ[i]);
        }
        M5.Lcd.drawString(freqLabel, x + barWidth/2, 225);
    }
    
    // 操作説明（固定部分）
    M5.Lcd.setTextColor(GREEN, BLACK);
    M5.Lcd.setTextDatum(TC_DATUM);
    M5.Lcd.drawString("A: FFT  B: Rec ON  C: Rec OFF", 160, 250);
    
    octaveBandScreenInitialized = true;
}

void updateOctaveBandBars()
{
    // CSV保存状態を更新（毎回更新）
    M5.Lcd.fillRect(60, 35, 200, 25, BLACK);  // CSV状態表示領域をクリア
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(csvLoggingEnabled ? GREEN : RED, BLACK);
    M5.Lcd.setTextDatum(TC_DATUM);
    char csvStatus[20];
    sprintf(csvStatus, "REC: %s", csvLoggingEnabled ? "ON" : "OFF");
    M5.Lcd.drawString(csvStatus, 160, 40);
    
    // バー部分のみ更新
    int barWidth = 35;
    int barSpacing = 40;
    int startX = 30;
    int barAreaTop = 80;    // バー描画領域の上端
    int barAreaBottom = 220; // バー描画領域の下端
    
    // 各バンドのバー領域をクリア・再描画
    for (int i = 0; i < OCTAVE_BAND_COUNT; i++) {
        int x = startX + i * barSpacing;
        float leqLevel = lastOctaveBandData.bandLeq[i];  // Leq値を使用
        
        // バー領域をクリア（黒で塗りつぶし）
        M5.Lcd.fillRect(x, barAreaTop, barWidth, barAreaBottom - barAreaTop, BLACK);
        
        // グリッド線を再描画（バー領域内のみ）
        M5.Lcd.setTextColor(CYAN, BLACK);
        for (int db = 40; db <= 120; db += 20) {
            int y = 220 - ((db - 40) * 140 / 80);
            if (y >= barAreaTop && y <= barAreaBottom) {
                M5.Lcd.drawLine(x, y, x + barWidth, y, DARKGREY);
            }
        }
        
        // バーの高さ計算 (40dB〜100dBの範囲)
        float constrainedLevel = constrain(leqLevel, 40.0, 120.0);
        int barHeight = 5;  // 最小の高さ
        if (leqLevel > 40.0) {
            barHeight = (constrainedLevel - 40.0) * 140 / 80;  // 140pixelの範囲
            if (barHeight < 5) barHeight = 5;
        }
        
        int y = 220 - barHeight;
        
        // バーの色を決定
        uint16_t barColor = GREEN;
        if (i == lastOctaveBandData.peakBandIndex && leqLevel > 40.0) {
            barColor = RED;  // ピークバンドは赤色
        } else if (leqLevel > 80.0) {
            barColor = YELLOW;
        } else if (leqLevel > 60.0) {
            barColor = ORANGE;
        }
        
        // バー描画
        if (y < barAreaBottom) {  // 描画範囲内の場合のみ
            M5.Lcd.fillRect(x, y, barWidth, barHeight, barColor);
            M5.Lcd.drawRect(x, y, barWidth, barHeight, WHITE);
        }
        
        // Leq値表示（バーの上部、数値表示領域をクリア）
        M5.Lcd.fillRect(x, barAreaTop - 20, barWidth, 15, BLACK);  // 数値表示領域をクリア
        if (leqLevel > 40.0) {
            M5.Lcd.setTextColor(WHITE, BLACK);
            M5.Lcd.setTextSize(1);
            M5.Lcd.setTextDatum(TC_DATUM);
            char leqLabel[8];
            sprintf(leqLabel, "%.0f", leqLevel);
            M5.Lcd.drawString(leqLabel, x + barWidth/2, y - 15);
        }
    }
}

void showDateTimeSettingScreen() {
    // 初期値を2025年に固定
    int year = 2025;
    int month = 1;
    int day = 1;
    int hour = 0;
    int minute = 0;
    int second = 0;

    int selector = 0;
    bool redraw = true;
    
    // 長押し検出用変数
    unsigned long btnBPressStart = 0;
    bool btnBPressed = false;

    while (true) {
        M5.update();
        yield();
        delay(1);

        if (redraw) {
            M5.Lcd.fillScreen(BLACK);
            M5.Lcd.setTextColor(WHITE);
            M5.Lcd.setTextSize(2);
            M5.Lcd.setTextDatum(TC_DATUM);
            M5.Lcd.drawString("Set Date & Time", 160, 15);

            char dateStr[20];
            sprintf(dateStr, "%04d-%02d-%02d", year, month, day);
            char timeStr[20];
            sprintf(timeStr, "%02d:%02d:%02d", hour, minute, second);

            M5.Lcd.setTextSize(3);
            M5.Lcd.drawString(dateStr, 160, 70);
            M5.Lcd.drawString(timeStr, 160, 130);

            int underlineY = (selector < 3) ? 100 : 160;
            int underlineX[] = {70, 160, 210, 80, 140, 200};
            int underlineW[] = {80, 30, 30, 30, 30, 30};
            M5.Lcd.drawLine(underlineX[selector], underlineY,
                            underlineX[selector] + underlineW[selector], underlineY,
                            GREEN);

            M5.Lcd.setTextSize(1);
            M5.Lcd.drawString("A: Select  B: +/-  C: OK", 160, 220);
            redraw = false;
        }

        if (M5.BtnA.wasPressed()) {
            selector = (selector + 1) % 6;
            redraw = true;
        }

        // Bボタンの押下開始を検出
        if (M5.BtnB.wasPressed()) {
            btnBPressStart = millis();
            btnBPressed = true;
        }
        
        // Bボタンの離し検出
        if (M5.BtnB.wasReleased() && btnBPressed) {
            unsigned long pressDuration = millis() - btnBPressStart;
            bool isLongPress = (pressDuration >= 500);
            int delta = isLongPress ? -1 : 1;
            btnBPressed = false;

            switch (selector) {
                case 0:
                    year += delta;
                    if (year < 2000) year = 2099;
                    if (year > 2099) year = 2000;
                    break;
                case 1:
                    month += delta;
                    if (month < 1) month = 12;
                    if (month > 12) month = 1;
                    break;
                case 2:
                    day += delta;
                    if (day < 1) day = 31;
                    if (day > 31) day = 1;
                    break;
                case 3:
                    hour = (hour + delta + 24) % 24;
                    break;
                case 4:
                    minute = (minute + delta + 60) % 60;
                    break;
                case 5:
                    second = (second + delta + 60) % 60;
                    break;
            }
            redraw = true;
        }

        if (M5.BtnC.wasPressed()) {
            struct tm t;
            t.tm_year = year - 1900;
            t.tm_mon = month - 1;
            t.tm_mday = day;
            t.tm_hour = hour;
            t.tm_min = minute;
            t.tm_sec = second;
            t.tm_isdst = 0;

            time_t newTime = mktime(&t);
            struct timeval now = { .tv_sec = newTime };
            settimeofday(&now, NULL);

            M5.Lcd.fillScreen(BLACK);
            M5.Lcd.setTextColor(GREEN);
            M5.Lcd.setTextSize(2);
            M5.Lcd.setTextDatum(TC_DATUM);
            M5.Lcd.drawString("Time Set!", 160, 120);
            delay(1000);
            return;
        }
    }
}

void showInstantPeakInfo()
{
    // 画面をクリア
    M5.Lcd.fillScreen(BLACK);
    
    // タイトル
    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextDatum(TC_DATUM);
    M5.Lcd.drawString("Instant Peak Analysis", 160, 20);
    
    // ピーク周波数表示
    M5.Lcd.setTextSize(3);
    M5.Lcd.setTextColor(YELLOW, BLACK);
    M5.Lcd.setTextDatum(TL_DATUM);
    
    char freqStr[32];
    sprintf(freqStr, "Frequency:");
    M5.Lcd.drawString(freqStr, 20, 70);
    
    M5.Lcd.setTextColor(CYAN, BLACK);
    sprintf(freqStr, "%.0f Hz", instantOctaveBandData.peakFrequency);
    M5.Lcd.drawString(freqStr, 20, 105);
    
    // dB値表示
    M5.Lcd.setTextColor(YELLOW, BLACK);
    char dbStr[32];
    sprintf(dbStr, "Level:");
    M5.Lcd.drawString(dbStr, 20, 150);
    
    M5.Lcd.setTextColor(CYAN, BLACK);
    sprintf(dbStr, "%.1f dB SPL", instantOctaveBandData.peakDB);
    M5.Lcd.drawString(dbStr, 20, 185);
    
    // 瞬時ピーク画面の操作説明を修正
    M5.Lcd.setTextSize(1);
    M5.Lcd.setTextColor(GREEN, BLACK);
    M5.Lcd.setTextDatum(TC_DATUM);
    M5.Lcd.drawString("A: Return to FFT  B: Return to Octave", 160, 250);
    M5.Lcd.drawString("C: New Peak Measurement", 160, 265);
}

void initCSVFileIfNeeded() {
    if (!SD.exists(CSV_FILE)) {
        File file = SD.open(CSV_FILE, FILE_WRITE);
        if (file) {
            // ヘッダー行に秒数を含める
            file.print("Date,Time");
            for (int i = 0; i < 7; i++) {
                file.print(",");
                file.print(freqs[i]);
                file.print("Hz");
            }
            file.println();
            file.close();
            Serial.println("CSV header created");
        } else {
            Serial.println("Failed to create CSV file");
        }
    }
}

void saveLeqToCSV(float leqValues[7]) {
    initCSVFileIfNeeded();

    File file = SD.open(CSV_FILE, FILE_APPEND);
    if (file) {
        time_t now = time(NULL);
        struct tm* t = localtime(&now);

        // 日付、時刻を別々のカラムで保存
        char dateStr[12];
        char timeStr[10];
        sprintf(dateStr, "%04d-%02d-%02d", t->tm_year + 1900, t->tm_mon + 1, t->tm_mday);
        sprintf(timeStr, "%02d:%02d:%02d", t->tm_hour, t->tm_min, t->tm_sec);
        
        // CSVフォーマット: 日付,時刻,63Hz,125Hz,250Hz,500Hz,1000Hz,2000Hz,4000Hz
        file.print(dateStr);
        file.print(",");
        file.print(timeStr);

        for (int i = 0; i < 7; i++) {
            file.print(",");
            file.print(leqValues[i], 1);
        }
        file.println();
        file.close();
        
        // シリアル出力でデバッグ（保存間隔の確認用）
        Serial.printf("CSV saved [%s %s]: ", dateStr, timeStr);
        for (int i = 0; i < 7; i++) {
            Serial.printf("%.1f", leqValues[i]);
            if (i < 6) Serial.print(",");
        }
        Serial.println();
    } else {
        Serial.println("Failed to open CSV file for writing");
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Starting PDM microphone with Octave Band Leq Analysis...");
    Serial.printf("Sample Rate: %d Hz, FFT Size: %d, Nyquist: %d Hz\n", 
                  SAMPLE_RATE, FFT_SIZE, NYQUIST_FREQ);
    
    M5.begin(true, true, true, true);
    header("Real-time FFT Spectrum", BLACK);
    microPhoneSetup();
    
    // 初期化
    for (int i = 0; i < OCTAVE_BAND_COUNT; i++) {
        lastOctaveBandData.bandLevels[i] = -100.0;
        lastOctaveBandData.bandLeq[i] = -100.0;
        instantOctaveBandData.bandLevels[i] = -100.0;
        instantOctaveBandData.bandLeq[i] = -100.0;
        
        // Leqバッファ初期化
        for (int j = 0; j < 29; j++) {  // 29サンプルで1秒
            leqBuffer[i][j] = 0.0;
        }
    }
    lastOctaveBandData.peakFrequency = 0;
    lastOctaveBandData.peakDB = -100.0;
    lastOctaveBandData.newDataAvailable = false;
    
    instantOctaveBandData.peakFrequency = 0;
    instantOctaveBandData.peakDB = -100.0;
    instantOctaveBandData.newDataAvailable = false;
    
    // 操作説明を画面に表示
    M5.Lcd.setTextSize(1);
    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.setTextDatum(TC_DATUM);
    M5.Lcd.drawString("A: FFT  B: Octave  C: Time Set", 160, 200);
    
    Serial.println("Button Functions:");
    Serial.println("A: Real-time FFT Spectrum");
    Serial.println("B: 1/1 Octave Band + CSV Auto-Save Toggle (1sec interval)");
    Serial.println("C: Set Date/Time");

    if (!SD.begin()) {
        M5.Lcd.println("SD init failed!");
        Serial.println("SD Card initialization failed!");
        while (1);  // SDエラーで停止
    }
    M5.Lcd.println("SD OK");
    Serial.println("SD Card OK");

    // 時刻設定
    configTime(9 * 3600, 0, "ntp.nict.jp");  // JST
    M5.Lcd.println("RTC Ready");
    Serial.println("RTC Ready");
}

void loop() {
    M5.update();
    
    // オクターブバンドデータの更新
    OctaveBandData_t newOctaveBandData;
    if (xQueueReceive(octaveBandQueue, &newOctaveBandData, 0) == pdTRUE) {
        lastOctaveBandData = newOctaveBandData;
    }
    
    // ボタン処理
    if (M5.BtnA.wasPressed()) {
        // A: リアルタイムFFT表示
        currentDisplayMode = MODE_REALTIME_FFT;
        csvLoggingEnabled = false;  // CSV保存を停止（A画面ではcsv保存はされない）
        Serial.println("A Button: Real-time FFT mode (CSV logging disabled)");
        header("Real-time FFT Spectrum", BLACK);
        octaveBandScreenInitialized = false;  // 次回B画面で再初期化
        M5.Lcd.setTextSize(1);
        M5.Lcd.setTextColor(WHITE, BLACK);
        M5.Lcd.setTextDatum(TC_DATUM);
        M5.Lcd.drawString("A: FFT  B: Octave  C: Time Set", 160, 200);
    }

    if (M5.BtnB.wasPressed()) {
        if (currentDisplayMode == MODE_OCTAVE_BAND) {
            // B画面でBボタン：CSV保存を再開
            csvLoggingEnabled = true;
            octaveBandScreenInitialized = false;  // 画面再描画
            Serial.println("B Button: CSV logging enabled");
        } else {
            // 他の画面でBボタン：B画面に切り替え
            currentDisplayMode = MODE_OCTAVE_BAND;
            octaveBandScreenInitialized = false;  // 画面初期化フラグをリセット
            Serial.println("B Button: Switched to Octave Band Display Mode");
        }
    }

    
    if (currentDisplayMode == MODE_OCTAVE_BAND && csvLoggingEnabled) {
        unsigned long now = millis();
        if (now - lastSaveTime >= SAVE_INTERVAL_MS) {
            saveLeqToCSV(leqResults);
            lastSaveTime = now;
        }
    }
    
    if (M5.BtnC.wasPressed()) {
        if (currentDisplayMode == MODE_OCTAVE_BAND) {
            // B画面でCボタン：CSV保存を停止
            csvLoggingEnabled = false;
            octaveBandScreenInitialized = false;  // 画面再描画
            Serial.println("C Button: CSV logging disabled");
        } else {
            // 他の画面でCボタン：時刻設定画面
            currentDisplayMode = MODE_SET_TIME;
            Serial.println("C Button: Date/Time setting");
            showDateTimeSettingScreen();
            currentDisplayMode = MODE_REALTIME_FFT; // 設定後はリアルタイムFFTに戻る
            header("Real-time FFT Spectrum", BLACK);
            M5.Lcd.setTextSize(1);
            M5.Lcd.setTextColor(WHITE, BLACK);
            M5.Lcd.drawString("A: FFT  B: Octave  C: Time Set", 10, 200);
        }
    }
    
    // 表示モードに応じた処理
    switch (currentDisplayMode) {
        case MODE_REALTIME_FFT:
            MicroPhoneFFT();
            break;
            
        case MODE_OCTAVE_BAND:
            // オクターブバンド画面では初期化後、1秒ごとにバー更新
            if (!octaveBandScreenInitialized) {
                initOctaveBandScreen();
                lastLeqUpdateTime = millis();  // 初期化時にタイマーリセット
            } else {
                unsigned long currentTime = millis();
                if (currentTime - lastLeqUpdateTime >= 1000) {  // 1秒ごとに更新
                    updateOctaveBandBars();
                    lastLeqUpdateTime = currentTime;
                }
            }
            break;
            
        case MODE_SHOW_PEAK_INFO:
            // ピーク情報表示中は手動で戻る必要がある
            // どのボタンも押されていない場合は何もしない
            break;
    }
    
    // CSV自動保存の実行（全モードで動作）
    if (csvLoggingEnabled) {
        unsigned long currentTime = millis();
        if (currentTime - lastSaveTime >= SAVE_INTERVAL_MS) {
            saveLeqToCSV(leqResults);
            lastSaveTime = currentTime;
            
            // 保存状況をシリアルで確認（デバッグ用）
            static int saveCount = 0;
            saveCount++;
            if (saveCount % 10 == 0) {  // 10回に1回表示
                Serial.printf("Auto-saved %d times\n", saveCount);
            }
        }
    }
    
    delay(10);
}