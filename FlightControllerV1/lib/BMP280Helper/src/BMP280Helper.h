#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

class BMP280Helper {
public:
    BMP280Helper(uint8_t address = 0x76); 
    bool begin();  
    void calibrateBaseHeight(float seaLevelPressure);
    float getRelativeHeight(float seaLevelPressure);
    
    uint8_t getSensorID();

private:
    Adafruit_BMP280 bmp; 
    uint8_t i2cAddress;
    float baseHeight;
};
