#include <Wire.h>
#include "MPU6050Helper.h"
#include "BMP280Helper.h"

// Pin configuration for MPU6050
#define SDA_PIN 18
#define SCL_PIN 19

// Constants for complementary filter tuning
#define GYRO_WEIGHT 0.98  // Gyroscope weight
#define ACCEL_WEIGHT 0.02 // Accelerometer weight

#define BMP280_I2C_ADDRESS 0x76
#define SEA_LEVEL_PRESSURE 1013.25 // hPa

MPU6050Helper mpu(SDA_PIN, SCL_PIN, 0.024, 0.06, GYRO_WEIGHT, ACCEL_WEIGHT);
BMP280Helper bmp(BMP280_I2C_ADDRESS);

void setup() {
    Serial.begin(115200);

    if (!mpu.begin()) {
        Serial.println("Failed to initialize MPU6050!");
        while (1); 
    } else {
        Serial.println("MPU6050 initialized!");
    }

    if (!bmp.begin()) {
        Serial.println("Failed to initialize BMP280!");
        Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                         "try a different address!"));
        while (1) delay(10);
    } else {
        Serial.println("BMP280 initialized!");
    }

    bmp.calibrateBaseHeight(SEA_LEVEL_PRESSURE);        //Calibrate current height as baseline
    Serial.println("BMP280 base height calibrated.");
}

void loop() {
    mpu.update();

    float gyroAngleX = mpu.getAngleX();
    float gyroAnglelY = mpu.getAngleY();
    float accelVelocityX = mpu.getVelocityX(); // Get smoothed accelerometer X velocity
    float accelVelocityY = mpu.getVelocityY(); // Get smoothed accelerometer Y velocity

    // Calculate accelerometer-based angle
    float accelAngleX = atan2(accelVelocityY, accelVelocityX);

    // Complementary filter to combine gyro and accelerometer
    float rollAngle = ACCEL_WEIGHT * accelAngleX + GYRO_WEIGHT * gyroAngleX;

    float relativeHeight = bmp.getRelativeHeight(SEA_LEVEL_PRESSURE);

    Serial.print("Roll Angle: ");
    Serial.print(rollAngle);
    Serial.print("° | Relative Height: ");
    Serial.print(relativeHeight);
    Serial.println(" m");

    delay(100);
}
