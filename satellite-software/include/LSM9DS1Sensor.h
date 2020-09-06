#ifndef __LSM9DS1Sensor__
#define __LSM9DS1Sensor__
#include <Adafruit_LSM9DS1.h>
#include "SensorBase.h"

//
// LSM9DS1 Sensor
//

class LSM9DS1Sensor : public SensorBase {
public:
    typedef enum {
        ACCELERATION_SENSITIVITY_2G = 0x01,
        ACCELERATION_SENSITIVITY_4G = 0x02,
        ACCELERATION_SENSITIVITY_8G = 0x03,
        ACCELERATION_SENSITIVITY_16G = 0x04
    } AccelerationSensitivitySetting;

    typedef enum {
        GYRO_SENSITIVITY_245DPS = 0x10,
        GYRO_SENSITIVITY_500DPS = 0x20,
        GYRO_SENSITIVITY_2000DPS = 0x30
    } GyroSensitivitySetting;
    
    typedef enum {
        MAGNETIC_SENSITIVITY_4GAUSS = 0x01,
        MAGNETIC_SENSITIVITY_8GAUSS = 0x02,
        MAGNETIC_SENSITIVITY_12GAUSS = 0x03,
        MAGNETIC_SENSITIVITY_16GAUSS = 0x04
    } MagneticSensitivitySetting;

private:
    Adafruit_LSM9DS1 _lsm;
    AccelerationSensitivitySetting _accelConfig;
    GyroSensitivitySetting _gyroConfig;
    MagneticSensitivitySetting _magConfig;   

    uint8_t _buffer[20];

    void setSensorConfig(void);
    void setSensorValueAtBufferLocation(float sensor_value, uint8_t index);
public:
    LSM9DS1Sensor();
    virtual ~LSM9DS1Sensor();

    virtual void setup(void)                                    { this->setSensorConfig(); }
    virtual const uint8_t* getCurrentMeasurementBuffer(void);
    virtual uint8_t getMeasurementBufferSize() const            { return 20; }
};

#endif