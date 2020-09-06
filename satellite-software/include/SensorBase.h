#ifndef __SensorBase__
#define __SensorBase__
#include <Arduino.h>

class SensorBase {
    static bool _isI2CSetUp;

public:
    SensorBase();
    virtual ~SensorBase();

    virtual void setup(void)    {}

    // This functions is expected to get a current sensor measurement
    // the fill a byte buffer that weill be transmitted via the 
    // LoRaWAN radio. It is the sensor's responsibility to define the
    // the buffer format for it's measurement. 
    // IMPORTANT: Returns NULL if sensor read was not successful.
    virtual const uint8_t* getCurrentMeasurementBuffer(void) = 0;

    virtual uint8_t getMeasurementBufferSize() const = 0;
};



#endif // __SensorBase__