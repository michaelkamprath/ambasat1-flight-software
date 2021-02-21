#ifndef __LSM9DS1Sensor__
#define __LSM9DS1Sensor__
#include "SensorBase.h"
#include "PersistedConfiguration.h"

//
// LSM9DS1 Sensor
//
#define LSM9DS1_ADDRESS                 0x6b
#define LSM9DS1_RESULTS_BUFFER_SIZE     20

//
// Configuration ENUMs
//

// LSM9DS1 Config
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


class LSM9DS1Sensor : public SensorBase {

private:
    uint8_t _buffer[LSM9DS1_RESULTS_BUFFER_SIZE];
    bool _continuousMode;

    AccelerationSensitivitySetting _accelSensitivity;
    GyroSensitivitySetting _gyroSensitivity;
    MagneticSensitivitySetting _magneticSensitivity;

    bool begin(void);
    void end(void);

    void setAccelFS(AccelerationSensitivitySetting config);
    void setGyroFS(GyroSensitivitySetting config);
    void setMagnetFS(MagneticSensitivitySetting config);

    void setSensorConfig(void);
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
    virtual uint8_t getMeasurementBufferSize() const            { return LSM9DS1_RESULTS_BUFFER_SIZE; }
    virtual uint8_t getPort() const                             { return 2; }

    //
    // Sensor Configuration Delegate
    //

    virtual uint8_t configBlockSize( void ) const;
    virtual void setDefaultValues(void);
    virtual void loadConfigValues(void);
    virtual void writeConfigToBuffer( uint8_t* bufferBaseAddress) const;

    AccelerationSensitivitySetting getAcceleratonSensitivitySetting(void) const     { return _accelSensitivity; }
    void setAcceleratonSensitivitySetting(AccelerationSensitivitySetting seting);

    GyroSensitivitySetting getGysroSensitivitySetting(void) const                   { return _gyroSensitivity; }
    void setGysroSensitivitySetting(GyroSensitivitySetting setting);

    MagneticSensitivitySetting getMagneticSensitivitySetting(void) const            { return _magneticSensitivity; }
    void setMagneticSensitivitySetting(MagneticSensitivitySetting setting);

#ifdef ENABLE_AMBASAT_COMMANDS
    // handles a command payload.
    virtual uint8_t handleCommand(uint16_t cmdSequenceID, uint8_t command, uint8_t* recievedData, uint8_t recievedDataLen);

#endif
};

#endif