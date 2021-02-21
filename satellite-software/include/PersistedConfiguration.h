#ifndef __PersistedConfiguration__
#define __PersistedConfiguration__
#include "AmbaSat1Config.h"

//
// Configuration ENUMs and defines
//

#define SATTELITE_PAYLOAD           0
#define LSM9DS1_PAYLOAD             1
#define MISSION_SENSOR_PAYLOAD      2
typedef int8_t UplinkPayloadType;

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
    uint8_t _uplinkPattern;
    UplinkPayloadType _lastPayloadUplinked;
    uint8_t _uplinkRateValue;

    uint8_t configBlockSize(void) const;

    bool isEEPROMErased(void) const;
    void loadAllCongigurations(void);
    uint32_t getCRC(void) const;
    bool checkCRC(void) const;

    // this setter is private because no one should be setting it outside this class.
    void setRebootCount(uint32_t rebootCount);
public:
    PersistedConfiguration();
    void init(void);

    void resetToDefaults(void);
    void updateCRC(void);

    void setSensorConfigDelegates(SensorConfigurationDelegate* LSM9DS1Delegate, SensorConfigurationDelegate* missionSensorDelegate);

    //
    // config items
    //

    uint32_t getRebootCount(void) const                     { return _rebootCount; }

    uint32_t getUplinkFrameCount(void) const                { return _uplinkFrameCount; }
    void setUplinkFrameCount(uint32_t frameCount);

    uint8_t getUplinkPattern(void) const                    { return _uplinkPattern; }
    void setUplinkPattern(uint8_t pattern);

    UplinkPayloadType getLastPayloadUplinked(void) const    { return _lastPayloadUplinked; }
    void setLastPayloadUplinked(UplinkPayloadType payload);

    uint8_t getUplinkSleepCycles(void) const                { return _uplinkRateValue; }
    void setUplinkSleepCycles(uint8_t rateValue);
};





#endif // __PersistedConfiguration__