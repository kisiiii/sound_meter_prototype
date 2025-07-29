#include "audio_processing.h"
#include "../modules/octave_band.h"

// arduinoFFT関連
ArduinoFFT<float> FFT = ArduinoFFT<float>();
float vReal[FFT_SIZE];
float vImag[FFT_SIZE];
bool captureInstantPeak = false;  // 瞬時値取得フラグ

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