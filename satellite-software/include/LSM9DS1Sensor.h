#ifndef __LSM9DS1Sensor__
#define __LSM9DS1Sensor__
#include "SensorBase.h"

//
// LSM9DS1 Sensor
//
#define LSM9DS1_ADDRESS            0x6b

class LSM9DS1Sensor : public SensorBase {
public:
    typedef enum {
        ACCELERATION_SENSITIVITY_OFF = 0x00,
        ACCELERATION_SENSITIVITY_2G = 0x01,
        ACCELERATION_SENSITIVITY_4G = 0x02,
        ACCELERATION_SENSITIVITY_8G = 0x03,
        ACCELERATION_SENSITIVITY_16G = 0x04
    } AccelerationSensitivitySetting;

    typedef enum {
        GYRO_SENSITIVITY_OFF = 0x00,
        GYRO_SENSITIVITY_245DPS = 0x10,
        GYRO_SENSITIVITY_500DPS = 0x20,
//        GYRO_SENSITIVITY_1000DPS = 0x30,
        GYRO_SENSITIVITY_2000DPS = 0x40
    } GyroSensitivitySetting;
    
    typedef enum {
        MAGNETIC_SENSITIVITY_OFF = 0x00,
        MAGNETIC_SENSITIVITY_4GAUSS = 0x01,
        MAGNETIC_SENSITIVITY_8GAUSS = 0x02,
        MAGNETIC_SENSITIVITY_12GAUSS = 0x03,
        MAGNETIC_SENSITIVITY_16GAUSS = 0x04
    } MagneticSensitivitySetting;

private:
    AccelerationSensitivitySetting _accelConfig;
    GyroSensitivitySetting _gyroConfig;
    MagneticSensitivitySetting _magConfig;   

    uint8_t _buffer[20];
    bool _continuousMode;

    bool begin(void);
    void end(void);
 
    int setAccelFS(AccelerationSensitivitySetting config);
    int setGyroFS(GyroSensitivitySetting config);
    int setMagnetFS(MagneticSensitivitySetting config);
    
    void setSensorConfig(void);
    void setSensorValueAtBufferLocation(float sensor_value, uint8_t index);
protected:

    // this only works for the acceleration and gyro part of the sensor.
    // normally should use explicit addressing in i2c calls.
    virtual uint8_t i2cDeviceAddress(void) const                { return 0x6b; }

public:
    LSM9DS1Sensor();
    virtual ~LSM9DS1Sensor();

    virtual void setup(void)                                    { this->setSensorConfig(); }
    virtual const uint8_t* getCurrentMeasurementBuffer(void);
    virtual uint8_t getMeasurementBufferSize() const            { return 20; }
    virtual uint8_t getPort() const                             { return 2; }
};

#endif