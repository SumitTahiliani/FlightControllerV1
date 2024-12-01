#ifndef MPU6050HELPER_H
#define MPU6050HELPER_H

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <deque> 

class MPU6050Helper {
public:
    MPU6050Helper(uint8_t sdaPin, uint8_t sclPin, float gyroXOffset, float gyroYOffset, float gyroAlpha, float accelAlpha);

    bool begin();

    void update();

    float getAngleX() const;
    float getAngleY() const;
    float getVelocityX() const;
    float getVelocityY() const;

private:
    uint8_t sdaPin;
    uint8_t sclPin;
    float gyroXOffset;
    float gyroYOffset;
    float gyroAlpha;
    float accelAlpha;

    Adafruit_MPU6050 mpu;

    float currentAngleX;
    float currentAngleY;
    float velocityX;
    float velocityY;

    float lastGyroX;
    float lastGyroY;

    bool firstRun;
    unsigned long prevTime;

    std::deque<float> gyroXBuffer;
    std::deque<float> gyroYBuffer;
    std::deque<float> accelXBuffer;
    std::deque<float> accelYBuffer;

    float movingAverage(std::deque<float>& buffer, float newValue, size_t windowSize);
};

#endif
