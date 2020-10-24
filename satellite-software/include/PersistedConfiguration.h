#ifndef __PersistedConfiguration__
#define __PersistedConfiguration__
#include "AmbaSat1Config.h"

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

// BME680 Config

typedef enum {
    BME680_OVERSAMPLING_NONE = 0,
    BME680_OVERSAMPLING_1x = 0b001,
    BME680_OVERSAMPLING_2x = 0b010,
    BME680_OVERSAMPLING_4x = 0b011,
    BME680_OVERSAMPLING_8x = 0b100,
    BME680_OVERSAMPLING_16x = 0b101
} BME680SensorOversamplingSetting;

typedef enum {
    BME_FILTER_COEF_0   = 0,
    BME_FILTER_COEF_1   = 0b001,
    BME_FILTER_COEF_3   = 0b010,
    BME_FILTER_COEF_7   = 0b011,
    BME_FILTER_COEF_15  = 0b100,
    BME_FILTER_COEF_31  = 0b101,
    BME_FILTER_COEF_63  = 0b110,
    BME_FILTER_COEF_127 = 0b111
} BME680IIRFilterCoefSetting;

//
// Class
//

class PersistedConfiguration
{
private:
    //
    // satellite configuration
    //
    uint32_t _rebootCount;
    uint32_t _uplinkFrameCount;
    AccelerationSensitivitySetting _accelSensitivity;
    GyroSensitivitySetting _gyroSensitivity;
    MagneticSensitivitySetting _magneticSensitivity;

    //
    // Mission Sensor Configuration
    //
#if AMBASAT_MISSION_SENSOR == SENSOR_BME680
    BME680SensorOversamplingSetting _temperatureOversampling;
    BME680SensorOversamplingSetting _humidityOversampling;
    BME680SensorOversamplingSetting _pressureOversampling;
    BME680IIRFilterCoefSetting _iirCoefSetting;
    int16_t _gasProfile0_targetTemp;
    int16_t _gasProfile0_duration;

#endif //AMBASAT_MISSION_SENSOR


    bool isEEPROMErased(void) const;
    void loadAllCongigurations(void);
    uint32_t getCRC(void) const;
    void updateCRC(void);
    bool checkCRC(void) const;

    // this setter is private because no one should be setting it outside this class.
    void setRebootCount(uint32_t rebootCount, bool updateCRC = true);
public:
    PersistedConfiguration();

    void resetToDefaults(void);

    //
    // config items
    //

    uint32_t getRebootCount(void) const           { return _rebootCount; }

    uint32_t getUplinkFrameCount(void) const      { return _uplinkFrameCount; }
     void setUplinkFrameCount(uint32_t frameCount, bool updateCRC = true);   
    
    //
    // LSM9DS1
    //

    AccelerationSensitivitySetting getAcceleratonSensitivitySetting(void) const     { return _accelSensitivity; }
    void setAcceleratonSensitivitySetting(AccelerationSensitivitySetting seting, bool updateCRC = true);
    
    GyroSensitivitySetting getGysroSensitivitySetting(void) const                   { return _gyroSensitivity; }
    void setGysroSensitivitySetting(GyroSensitivitySetting setting, bool updateCRC = true);

    MagneticSensitivitySetting getMagneticSensitivitySetting(void) const            { return _magneticSensitivity; }
    void setMagneticSensitivitySetting(MagneticSensitivitySetting setting, bool updateCRC = true);

    //
    // Mission Sensor
    //

#if AMBASAT_MISSION_SENSOR == SENSOR_BME680
    // BME680
    BME680SensorOversamplingSetting getTemperatureOversampling(void) const          { return _temperatureOversampling; }
    void setTemperatureOversampling(BME680SensorOversamplingSetting setting, bool updateCRC = true);

    BME680SensorOversamplingSetting getHumidityOversampling(void) const             { return _humidityOversampling; }
    void setHumidityOversampling(BME680SensorOversamplingSetting setting, bool updateCRC = true);

    BME680SensorOversamplingSetting getPressureOversampling(void) const             { return _pressureOversampling; }
    void setPressureOversampling(BME680SensorOversamplingSetting setting, bool updateCRC = true);

    BME680IIRFilterCoefSetting getIIRFilterCoef(void) const                        { return _iirCoefSetting; }
    void setIIRFilterCoef(BME680IIRFilterCoefSetting setting, bool updateCRC = true);

    int16_t getGasHeatDuration(uint8_t profile = 0) const                           { return _gasProfile0_duration; }
    void setGasHeatDuration(int16_t setting, bool updateCRC = true, uint8_t profile = 0);

    int16_t getGasHeaterTemperature(uint8_t profile = 0) const                      { return _gasProfile0_targetTemp; }
    void setGasHeatTemperature(int16_t setting, bool updateCRC = true, uint8_t profile = 0);

#endif
};





#endif // __PersistedConfiguration__