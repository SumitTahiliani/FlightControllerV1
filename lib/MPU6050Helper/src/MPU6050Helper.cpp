#include "MPU6050Helper.h"
#include <cmath>
#include <deque> 

#define ROUND2(value) (round((value) * 100) / 100.0)

MPU6050Helper::MPU6050Helper(uint8_t sdaPin, uint8_t sclPin, float gyroXOffset, float gyroYOffset, float gyroAlpha, float accelAlpha)
    : sdaPin(sdaPin), sclPin(sclPin), gyroXOffset(gyroXOffset), gyroYOffset(gyroYOffset),
      gyroAlpha(gyroAlpha), accelAlpha(accelAlpha), currentAngleX(0), currentAngleY(0),
      velocityX(0), velocityY(0), lastGyroX(0), lastGyroY(0), firstRun(true), prevTime(0) {}

bool MPU6050Helper::begin() {
    Wire.begin(sdaPin, sclPin); 
    if (!mpu.begin()) {
        return false;
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);

    prevTime = millis();
    return true;
}

void MPU6050Helper::update() {
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);

    unsigned long currentTime = millis();

    if (firstRun) {
        prevTime = currentTime;
        firstRun = false;
        return;
    }

    float deltaTime = (currentTime - prevTime) / 1000.0;
    prevTime = currentTime;

    float gyroX = gyro.gyro.x - gyroXOffset;
    float gyroY = gyro.gyro.y - gyroYOffset;

    lastGyroX = movingAverage(gyroXBuffer, gyroX, 10);
    lastGyroY = movingAverage(gyroYBuffer, gyroY, 10);

    currentAngleX += lastGyroX * deltaTime;
    currentAngleY += lastGyroY * deltaTime;

    currentAngleX = ROUND2(currentAngleX);
    currentAngleY = ROUND2(currentAngleY);

    float smoothedAccelX = movingAverage(accelXBuffer, accel.acceleration.x, 10);
    float smoothedAccelY = movingAverage(accelYBuffer, accel.acceleration.y, 10);
    float smoothedAccelZ = movingAverage(accelZBuffer, accel.acceleration.z, 10 );

    velocityX += smoothedAccelX * deltaTime;
    velocityY += smoothedAccelY * deltaTime;
}

float MPU6050Helper::getAngleX() const {
    return currentAngleX;               
}

float MPU6050Helper::getAngleY() const {
    return currentAngleY;
}

float MPU6050Helper::getVelocityX() const {
    return velocityX;
}

float MPU6050Helper::getVelocityY() const {
    return velocityY;
}
float MPU6050Helper::getAccelX() const {
    return smoothedAccelX;
}
float MPU6050Helper::getAccelY() const {
    return smoothedAccelY;
}
float MPU6050Helper::getAccelZ() const {
    return smoothedAccelZ;
}

float MPU6050Helper::movingAverage(std::deque<float>& buffer, float newValue, size_t windowSize) {
    buffer.push_back(newValue); 
    if (buffer.size() > windowSize) {
        buffer.pop_front();
    }

    float sum = 0.0;
    for (float value : buffer) {
        sum += value;
    }
    return sum / buffer.size();
}
