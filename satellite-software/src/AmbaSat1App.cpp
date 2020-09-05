#include <lmic.h>
#include "AmbaSat1App.h"
#include "Utilities.h"

AmbaSat1App::AmbaSat1App()
    :   _voltSensor(),
        _lsm9DS1Sensor()
{


}

AmbaSat1App::~AmbaSat1App()
{

}

void AmbaSat1App::setup()
{
    // most set up was done during construction

}

void AmbaSat1App::loop() 
{
    // DEMO - get and print sensor buffers

    const uint8_t* bufferPtr = nullptr;

    Serial.print(F("Voltage buffer = "));
    print_buffer(
        _voltSensor.getCurrentMeasurementBuffer(),
        _voltSensor.getMeasurementBufferSize()
    );
    Serial.print(F("\n\n"));

    Serial.print(F("LSM9DS1 buffer = "));
    print_buffer(
        _lsm9DS1Sensor.getCurrentMeasurementBuffer(),
        _lsm9DS1Sensor.getMeasurementBufferSize()
    );
    Serial.print(F("\n\n"));
   
   delay(2000);
}
