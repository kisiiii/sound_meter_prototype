#include "octave_band.h"

// Leq計算用のバッファ（1秒間のデータを保存）
float leqBuffer[6][29];  // 29個のサンプル（1秒分：1024/30000*29≈1秒）
int leqBufferIndex = 0;
bool leqBufferFull = false;

// B画面の初期化フラグ
bool octaveBandScreenInitialized = false;
unsigned long lastLeqUpdateTime = 0;  // 1秒ごと更新用タイマー

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
    
    // LAeq合算計算
    calculateLAeq();
    
    // バッファインデックス更新
    leqBufferIndex++;
    if (leqBufferIndex >= 29) {  // 29サンプルで1秒
        leqBufferIndex = 0;
        leqBufferFull = true;
    }
}

// LAeq合算計算関数
void calculateLAeq() {
    float sumLinear = 0.0;
    
    for (int band = 0; band < OCTAVE_BAND_COUNT; band++) {
        // A特性補正を適用してリニア値に変換
        float aWeightedDB = leqResults[band] + A_WEIGHTING_VALUES[band];
        float linearValue = pow(10.0, aWeightedDB / 10.0);
        sumLinear += linearValue;
    }
    
    // 合算したリニア値をdBに変換
    if (sumLinear > 0) {
        laeqResult = 10.0 * log10(sumLinear);
    } else {
        laeqResult = -100.0;  // 極小値
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
            float rawDB = calculateDB(rmsValue);
            // 補正値を適用
            result->bandLevels[band] = rawDB + CORRECTION_VALUES[band];
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
    for (int db = 20; db <= 120; db += 20) {
        int y = 220 - ((db - 20) * 140 / 100);  // 20-120dBの範囲で140pixel
        char label[8];
        sprintf(label, "%d", db);
        M5.Lcd.drawString(label, 5, y - 4);
        M5.Lcd.drawLine(25, y, 310, y, DARKGREY);  // グリッド線
    }
    
    // 中心周波数ラベル（固定部分）- LAeqバーを含めて7本表示
    int barWidth = 32;
    int barSpacing = 40;
    int startX = 25;
    
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
    
    // LAeqラベルを最右に追加
    int laeqX = startX + OCTAVE_BAND_COUNT * barSpacing;
    M5.Lcd.drawString("LAeq", laeqX + barWidth/2, 225);
    
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
    int barWidth = 32;
    int barSpacing = 40;
    int startX = 25;
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
        for (int db = 20; db <= 120; db += 20) {
            int y = 220 - ((db - 20) * 140 / 100);
            if (y >= barAreaTop && y <= barAreaBottom) {
                M5.Lcd.drawLine(x, y, x + barWidth, y, DARKGREY);
            }
        }
        
        // バーの高さ計算 (20dB〜120dBの範囲)
        float constrainedLevel = constrain(leqLevel, 20.0, 120.0);
        int barHeight = 5;  // 最小の高さ
        if (leqLevel > 20.0) {
            barHeight = (constrainedLevel - 20.0) * 140 / 100;  // 140pixelの範囲
            if (barHeight < 5) barHeight = 5;
        }
        
        int y = 220 - barHeight;
        
        // バーの色を決定
        uint16_t barColor = GREEN;
        if (i == lastOctaveBandData.peakBandIndex && leqLevel > 20.0) {
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
        if (leqLevel > 20.0) {
            M5.Lcd.setTextColor(WHITE, BLACK);
            M5.Lcd.setTextSize(1);
            M5.Lcd.setTextDatum(TC_DATUM);
            char leqLabel[8];
            sprintf(leqLabel, "%.0f", leqLevel);
            M5.Lcd.drawString(leqLabel, x + barWidth/2, y - 15);
        }
    }
    
    // LAeqバーを最右に描画
    int laeqX = startX + OCTAVE_BAND_COUNT * barSpacing;
    float laeqLevel = laeqResult;
    
    // LAeqバー領域をクリア（黒で塗りつぶし）
    M5.Lcd.fillRect(laeqX, barAreaTop, barWidth, barAreaBottom - barAreaTop, BLACK);
    
    // グリッド線を再描画（LAeqバー領域内のみ）
    M5.Lcd.setTextColor(CYAN, BLACK);
    for (int db = 20; db <= 120; db += 20) {
        int y = 220 - ((db - 20) * 140 / 100);
        if (y >= barAreaTop && y <= barAreaBottom) {
            M5.Lcd.drawLine(laeqX, y, laeqX + barWidth, y, DARKGREY);
        }
    }
    
    // LAeqバーの高さ計算 (20dB〜120dBの範囲)
    float constrainedLAeq = constrain(laeqLevel, 20.0, 120.0);
    int laeqBarHeight = 5;  // 最小の高さ
    if (laeqLevel > 20.0) {
        laeqBarHeight = (constrainedLAeq - 20.0) * 140 / 100;  // 140pixelの範囲
        if (laeqBarHeight < 5) laeqBarHeight = 5;
    }
    
    int laeqY = 220 - laeqBarHeight;
    
    // LAeqバーの色を決定（常に紫色で目立たせる）
    uint16_t laeqBarColor = MAGENTA;
    
    // LAeqバー描画
    if (laeqY < barAreaBottom) {  // 描画範囲内の場合のみ
        M5.Lcd.fillRect(laeqX, laeqY, barWidth, laeqBarHeight, laeqBarColor);
        M5.Lcd.drawRect(laeqX, laeqY, barWidth, laeqBarHeight, WHITE);
    }
    
    // LAeq値表示（バーの上部、数値表示領域をクリア）
    M5.Lcd.fillRect(laeqX, barAreaTop - 20, barWidth, 15, BLACK);  // 数値表示領域をクリア
    if (laeqLevel > 20.0) {
        M5.Lcd.setTextColor(WHITE, BLACK);
        M5.Lcd.setTextSize(1);
        M5.Lcd.setTextDatum(TC_DATUM);
        char laeqLabel[8];
        sprintf(laeqLabel, "%.0f", laeqLevel);
        M5.Lcd.drawString(laeqLabel, laeqX + barWidth/2, laeqY - 15);
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