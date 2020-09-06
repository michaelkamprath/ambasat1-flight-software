#ifndef __VoltageSensor__
#define __VoltageSensor__
#include "SensorBase.h"

//
// Voltage Sensor
//

class VoltageSensor : public SensorBase {
private:
    uint8_t _buffer[2];

public:
    VoltageSensor();
    virtual ~VoltageSensor();

    virtual void setup(void);
    virtual const uint8_t* getCurrentMeasurementBuffer(void);
    virtual uint8_t getMeasurementBufferSize() const                    { return 2; }
};

#endif