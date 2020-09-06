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
    _voltSensor.setup();
    _lsm9DS1Sensor.setup();
}

void AmbaSat1App::loop() 
{
    // DEMO - get and print sensor buffers

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
