#ifndef __Si1132Sensor__
#define __Si1132Sensor__
#include "SensorBase.h"

class Si1132Sensor : public SensorBase
{
private:
    uint8_t _buffer[6];
    uint8_t _MEAS_RATE0;
    uint8_t _MEAS_RATE1;
    
    bool begin(void);
    bool waitUntilSleep(void);
    uint8_t readResponseRegister(void);
    bool sendCommand(uint8_t cmd_value);
    uint8_t setParameter(uint8_t param, uint8_t value);

public:
    Si1132Sensor();
    virtual ~Si1132Sensor();

    virtual void setup(void);
    virtual const uint8_t* getCurrentMeasurementBuffer(void);
    virtual uint8_t getMeasurementBufferSize() const            { return 6; }
    virtual uint8_t getPort() const                             { return 8; }
};



#endif //__Si1132Sensor__