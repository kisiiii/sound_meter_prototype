#include "time_setting.h"

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