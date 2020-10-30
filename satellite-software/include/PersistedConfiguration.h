#ifndef __PersistedConfiguration__
#define __PersistedConfiguration__
#include "AmbaSat1Config.h"

//
// Configuration ENUMs
//

//
// Sensor Configuration Interface
//
class PersistedConfiguration;

class SensorConfigurationDelegate {
private:
    uint16_t _baseEEPROMAddress;

protected:
    PersistedConfiguration& _config;

    uint16_t getEEPROMBaseAddress(void) const  { return _baseEEPROMAddress; }
public:
    SensorConfigurationDelegate(PersistedConfiguration& config);

    virtual uint8_t configBlockSize( void ) const  = 0;
    virtual void setDefaultValues(void) = 0;
    virtual void loadConfigValues(void) = 0;
    virtual void writeConfigToBuffer( uint8_t* bufferBaseAddress) const = 0;

    void setBaseEEPROMAddress(uint16_t address);
};

//
// Confiration Class
//

class PersistedConfiguration
{
private:
    SensorConfigurationDelegate* _LSM9DS1Delegate;
    SensorConfigurationDelegate* _missionSensorDelegate;
    //
    // satellite configuration
    //
    uint32_t _rebootCount;
    uint32_t _uplinkFrameCount;

    //
    // Mission Sensor Configuration
    //
#if AMBASAT_MISSION_SENSOR == SENSOR_BME680

#elif AMBASAT_MISSION_SENSOR == SENSOR_SI1132
    uint8_t _adcGainVisible;
    uint8_t _adcGainInfraRed;
    bool _highSignalVisible;
    bool _highSignalInfraRed;
#endif //AMBASAT_MISSION_SENSOR


    uint8_t configBlockSize(void) const;

    bool isEEPROMErased(void) const;
    void loadAllCongigurations(void);
    uint32_t getCRC(void) const;
    bool checkCRC(void) const;

    // this setter is private because no one should be setting it outside this class.
    void setRebootCount(uint32_t rebootCount, bool updateCRC = true);
public:
    PersistedConfiguration();
    void init(void);

    void resetToDefaults(void);
    void updateCRC(void);

    void setSensorConfigDelegates(SensorConfigurationDelegate* LSM9DS1Delegate, SensorConfigurationDelegate* missionSensorDelegate);

    //
    // config items
    //

    uint32_t getRebootCount(void) const           { return _rebootCount; }

    uint32_t getUplinkFrameCount(void) const      { return _uplinkFrameCount; }
    void setUplinkFrameCount(uint32_t frameCount, bool updateCRC = true);   
    


    //
    // Mission Sensor
    //

#if AMBASAT_MISSION_SENSOR == SENSOR_BME680

#elif AMBASAT_MISSION_SENSOR == SENSOR_SI1132
    // Si1132
    uint8_t getVisibleADCGain(void) const                                           { return _adcGainVisible; }
    void setVisibleADCGain(uint8_t setting, bool updateCRC = true);

    uint8_t getInfraRedADCGain(void) const                                          { return _adcGainInfraRed; }
    void setInfraRedADCGain(uint8_t setting, bool updateCRC = true);

    bool isVisibleHighSignalRange(void) const                                       { return _highSignalVisible; }
    void setIsVisibleHighSignalRange(bool setting, bool updateCRC = true);

    bool isInfraRedHighSignalRange(void) const                                      { return _highSignalInfraRed; }
    void setIsInfraRedHighSignalRange(bool setting, bool updateCRC = true);

#endif
};





#endif // __PersistedConfiguration__