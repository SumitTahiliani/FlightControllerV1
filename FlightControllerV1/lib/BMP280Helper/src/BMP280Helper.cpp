#include "BMP280Helper.h"

BMP280Helper::BMP280Helper(uint8_t address)
    : i2cAddress(address), baseHeight(0) {}

bool BMP280Helper::begin() {
    if (!bmp.begin(i2cAddress)) {
        return false;
    }

    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X1,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_125); /* Standby time. */
    return true;
}

void BMP280Helper::calibrateBaseHeight(float seaLevelPressure) {
    long startTime = millis();
    float sumHeight = 0;
    int count = 0;

    while (millis() - startTime < 2000) { // Collect data for 2 seconds
        sumHeight += bmp.readAltitude(seaLevelPressure);
        count++;
        delay(125);
    }

    baseHeight = sumHeight / count;
}

float BMP280Helper::getRelativeHeight(float seaLevelPressure) {
    float currentHeight = bmp.readAltitude(seaLevelPressure);
    return currentHeight - baseHeight;
}

uint8_t BMP280Helper::getSensorID() {
    return bmp.sensorID();
}
