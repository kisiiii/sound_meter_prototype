#include "fft_display.h"

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