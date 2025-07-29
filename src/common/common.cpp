#include "common.h"

// グローバル変数の定義
const unsigned long SAVE_INTERVAL_MS = 1000;  // 1秒間隔で保存（1000ms = 1秒）
const char* freqs[] = { "125", "250", "500", "1000", "2000", "4000" };
bool csvLoggingEnabled = false;  // CSV自動保存の有効/無効フラグ

// 1/1オクターブバンドの中心周波数 (125Hz〜4KHz)
const float OCTAVE_CENTER_FREQ[] = {125, 250, 500, 1000, 2000, 4000};
const int OCTAVE_BAND_COUNT = 6;

// 周波数帯域別補正値 (dB) - 125Hz, 250Hz, 500Hz, 1000Hz, 2000Hz, 4000Hz
const float CORRECTION_VALUES[] = {-20.2, -17.3, -14.5, -12.2, -14.3, -10.3};

// A特性補正値 (dB) - 125Hz, 250Hz, 500Hz, 1000Hz, 2000Hz, 4000Hz
const float A_WEIGHTING_VALUES[] = {-16.1, -8.6, -3.2, 0.0, 1.2, 1.0};

// Leq計算結果格納用
float leqResults[6] = {0};
float laeqResult = 0.0;  // LAeq合算値
unsigned long lastSaveTime = 0;

// 表示モード
DisplayMode currentDisplayMode = MODE_REALTIME_FFT;

// オクターブバンドデータ
OctaveBandData_t lastOctaveBandData;
OctaveBandData_t instantOctaveBandData;  // 瞬時値保存用

// スプライト・キュー
TFT_eSprite DisFFTbuff = TFT_eSprite(&M5.Lcd);
TFT_eSprite OctaveBandBuff = TFT_eSprite(&M5.Lcd);
QueueHandle_t fftvalueQueue = nullptr;
QueueHandle_t i2sstateQueue = nullptr;
QueueHandle_t octaveBandQueue = nullptr;

// 共通関数の実装

void header(const char *string, uint16_t color)
{
    M5.Lcd.fillScreen(color);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.fillRect(0, 0, 320, 30, BLACK);
    M5.Lcd.setTextDatum(TC_DATUM);
    M5.Lcd.drawString(string, 160, 3, 4); 
}

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

void initCSVFileIfNeeded() {
    if (!SD.exists(CSV_FILE)) {
        File file = SD.open(CSV_FILE, FILE_WRITE);
        if (file) {
            // ヘッダー行に秒数を含める
            file.print("Date,Time");
            for (int i = 0; i < 6; i++) {
                file.print(",");
                file.print(freqs[i]);
                file.print("Hz");
            }
            file.print(",LAeq");  // LAeq列を追加
            file.println();
            file.close();
            Serial.println("CSV header created");
        } else {
            Serial.println("Failed to create CSV file");
        }
    }
}

void saveLeqToCSV(float leqValues[6]) {
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

        for (int i = 0; i < 6; i++) {
            file.print(",");
            file.print(leqValues[i], 1);
        }
        file.print(",");
        file.print(laeqResult, 1);  // LAeq値を追加
        file.println();
        file.close();
        
        // シリアル出力でデバッグ（保存間隔の確認用）
        Serial.printf("CSV saved [%s %s]: ", dateStr, timeStr);
        for (int i = 0; i < 6; i++) {
            Serial.printf("%.1f", leqValues[i]);
            Serial.print(",");
        }
        Serial.printf("%.1f", laeqResult);  // LAeq値を追加
        Serial.println();
    } else {
        Serial.println("Failed to open CSV file for writing");
    }
}