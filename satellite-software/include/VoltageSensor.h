#ifndef __VoltageSensor__
#define __VoltageSensor__
#include "SensorBase.h"

class PersistedConfiguration;

//
// Voltage Sensor
//

class VoltageSensor : public SensorBase {
private:
    uint8_t _buffer[2];

    int16_t readVccMilliVolts(void) const;

protected:
    // this sensor is not i2c
    virtual uint8_t i2cDeviceAddress(void) const                        { return 0xFF; }
public:
    VoltageSensor(PersistedConfiguration& config);
    virtual ~VoltageSensor();

    virtual void setup(void);
    virtual const uint8_t* getCurrentMeasurementBuffer(void);
    virtual uint8_t getMeasurementBufferSize() const                    { return 2; }
    virtual uint8_t getPort() const                                     { return 1; }
};

#endif