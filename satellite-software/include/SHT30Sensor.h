#ifndef __SHT30Sensor__
#define __SHT30Sensor__
#include "SensorBase.h"


class SHT30Sensor : public SensorBase 
{
    uint8_t _buffer[3];

    bool begin(void);
    void reset(void);

    bool sendCommand(uint16_t cmd, bool acceptNACKAtEnd = false );
    uint16_t readStatus(void);
protected:
    // The SHT30 has no concept of register address autoincrement
    virtual uint8_t i2cAutoIncrementBit(void) const             { return 0; } 
    virtual uint8_t i2cDeviceAddress(void) const                { return 0x44; }
public:
    SHT30Sensor(PersistedConfiguration& config);
    virtual ~SHT30Sensor();

    virtual void setup(void);
    virtual bool isActive(void) const;
    virtual const uint8_t* getCurrentMeasurementBuffer(void);
    virtual uint8_t getMeasurementBufferSize() const            { return 3; }
    virtual uint8_t getPort() const                             { return 4; }

};

#endif // __SHT30Sensor__