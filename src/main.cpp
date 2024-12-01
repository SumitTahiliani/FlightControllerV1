#include <Wire.h>
#include "MPU6050Helper.h"
#include "BMP280Helper.h"

// Pin configuration for MPU6050
#define SDA_PIN 21
#define SCL_PIN 22

// Constants for complementary filter tuning
#define GYRO_WEIGHT 0.98  // Gyroscope weight
#define ACCEL_WEIGHT 0.02 // Accelerometer weight

#define BMP280_I2C_ADDRESS 0x76
#define SEA_LEVEL_PRESSURE 1013.25 // hPa

MPU6050Helper mpu(SDA_PIN, SCL_PIN, 0.024, -0.06, GYRO_WEIGHT, ACCEL_WEIGHT);
BMP280Helper bmp(BMP280_I2C_ADDRESS);

float roll = 0.0;
float pitch = 0.0;

void setup() {
    Serial.begin(115200);

    if (!mpu.begin()) {
        Serial.println("Failed to initialize MPU6050!");
        while (1) delay(10); 
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

    //current angle values equivalent to roll_rate * deltaTime
    float gyroRoll = mpu.getAngleX();      
    float gyroPitch = mpu.getAngleY();

    //last estimate + current_angle to feed into complimentary filter
    float gyroRollEstimate = gyroRoll + roll;      
    float gyroPitchEstimate = gyroPitch + pitch;

    //raw acceleretometer values in m/s2
    float accelX = mpu.getAccelX();
    float accelY = mpu.getAccelY();
    float accelZ = mpu.getAccelZ();
    float g = 9.81;

    // Calculate accelerometer-based angle
    float accelRoll = atan2(accelY, accelZ);
    float accelPitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ));

    // Complementary filter to combine gyro and accelerometer
    roll = GYRO_WEIGHT * gyroRollEstimate + ACCEL_WEIGHT * accelRoll;
    pitch = GYRO_WEIGHT * gyroPitchEstimate + ACCEL_WEIGHT * accelPitch;

    float relativeHeight = bmp.getRelativeHeight(SEA_LEVEL_PRESSURE);

    Serial.print("Roll (radians): ");
    Serial.println(roll); // Convert radians to degrees
    Serial.print(" | Pitch (radians): ");
    Serial.print(pitch); // Convert radians to degrees
    Serial.print(" | Relative Height: ");
    Serial.print(relativeHeight);
    Serial.println(" m");

    delay(100);
}
