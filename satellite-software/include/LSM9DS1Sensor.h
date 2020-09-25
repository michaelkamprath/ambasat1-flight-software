#ifndef __LSM9DS1Sensor__
#define __LSM9DS1Sensor__
#include "SensorBase.h"
#include "PersistedConfiguration.h"

//
// LSM9DS1 Sensor
//
#define LSM9DS1_ADDRESS            0x6b

class LSM9DS1Sensor : public SensorBase {

private:
    uint8_t _buffer[20];
    bool _continuousMode;

    bool begin(void);
    void end(void);
 
    void setAccelFS(AccelerationSensitivitySetting config);
    void setGyroFS(GyroSensitivitySetting config);
    void setMagnetFS(MagneticSensitivitySetting config);
    
    void setSensorConfig(void);
    void setSensorValueAtBufferLocation(float sensor_value, uint8_t index);
protected:
    virtual uint8_t i2cAutoIncrementBit(void) const         { return 7; }

    // this only works for the acceleration and gyro part of the sensor.
    // normally should use explicit addressing in i2c calls.
    virtual uint8_t i2cDeviceAddress(void) const                { return 0x6b; }

public:
    LSM9DS1Sensor(PersistedConfiguration& config);
    virtual ~LSM9DS1Sensor();

    virtual void setup(void)                                    { this->setSensorConfig(); }
    virtual bool isActive(void) const;
    virtual const uint8_t* getCurrentMeasurementBuffer(void);
    virtual uint8_t getMeasurementBufferSize() const            { return 20; }
    virtual uint8_t getPort() const                             { return 2; }
};

#endif