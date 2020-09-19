#ifndef __PersistedConfiguration__
#define __PersistedConfiguration__

//
// Configuration ENUMs
//

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

//
// Class
//

class PersistedConfiguration
{
private:
    uint32_t _rebootCount;
    uint32_t _uplinkFrameCount;
    AccelerationSensitivitySetting _accelSensitivity;
    GyroSensitivitySetting _gyroSensitivity;
    MagneticSensitivitySetting _magneticSensitivity;

    bool isEEPROMErased(void) const;
    void loadAllCongigurations(void);
    uint32_t calculateCRC(void) const;
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
    
    AccelerationSensitivitySetting getAcceleratonSensitivitySetting(void) const     { return _accelSensitivity; }
    void setAcceleratonSensitivitySetting(AccelerationSensitivitySetting seting, bool updateCRC = true);
    
    GyroSensitivitySetting getGysroSensitivitySetting(void) const                   { return _gyroSensitivity; }
    void setGysroSensitivitySetting(GyroSensitivitySetting setting, bool updateCRC = true);

    MagneticSensitivitySetting getMagneticSensitivitySetting(void) const            { return _magneticSensitivity; }
    void setMagneticSensitivitySetting(MagneticSensitivitySetting setting, bool updateCRC = true);

};





#endif // __PersistedConfiguration__