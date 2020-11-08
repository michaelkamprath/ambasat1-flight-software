#ifndef __LoRaPayloadBase__
#define __LoRaPayloadBase__
#include <Arduino.h>

// 
// This class is really just an interface definition
//
class LoRaPayloadBase {

public:
    // This functions is expected to get a current payload measurement then fill a 
    // byte buffer that will be transmitted via the  LoRaWAN radio.  It is the 
    // sensor's responsibility to define the the buffer format for it's measurement. 
    // IMPORTANT: Returns NULL if sensor read was not successful.
    virtual const uint8_t* getCurrentMeasurementBuffer(void) = 0;

    // The number of bytes returned by getCurrentMeasurementBuffer()
    virtual uint8_t getMeasurementBufferSize() const = 0;

    // defines the LoRaWAN port the payload should be sent across.
    virtual uint8_t getPort() const = 0;

#ifdef ENABLE_AMBASAT_COMMANDS
    // handles a command payload. 
    virtual void handleCommand(uint8_t* recievedData, uint8_t recievedDataLen)          { ; }

#endif
};

#endif // __LoRaPayloadBase__