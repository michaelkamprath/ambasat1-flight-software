#ifndef __LoggerSensor__
#define __LoggerSensor__
#include "SensorBase.h"
#include "PersistedConfiguration.h"


class LoggerSensor : public SensorBase {

private:
    char logMessage[255] = "";

protected:
    virtual uint8_t i2cAutoIncrementBit(void) const             { return -1; }
    virtual uint8_t i2cDeviceAddress(void) const                { return -1; }

public:
    LoggerSensor(PersistedConfiguration& config);
    virtual ~LoggerSensor();

    virtual bool isActive(void) const;
    const uint8_t* getCurrentMeasurementBuffer(void)     {return ((uint8_t *)logMessage); }
    virtual const void setlogMessage(char[]);
    uint8_t getMeasurementBufferSize() const;
    uint8_t getPort() const                             { return 12; }    
    virtual uint8_t configBlockSize( void ) const;
    virtual void setDefaultValues(void);
    virtual void loadConfigValues(void);
    virtual void writeConfigToBuffer( uint8_t* bufferBaseAddress) const;
};

#endif