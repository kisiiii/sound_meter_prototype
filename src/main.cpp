#include "common/common.h"
#include "modules/fft_display.h"
#include "modules/octave_band.h"
#include "modules/time_setting.h"
#include "audio/audio_processing.h"

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