#include "LoggerSensor.h"
#include "Logging.h"
#include "Utilities.h"
#include "PersistedConfiguration.h"

LoggerSensor::LoggerSensor(PersistedConfiguration& config)
    :   SensorBase(config)
{
}

LoggerSensor::~LoggerSensor()
{
}

const void LoggerSensor::setlogMessage(char message[]) 
{
    int i = 0;
    while(message[i] != 0x0) {
        logMessage[i] = message[i];
        i++;
    }
}

uint8_t LoggerSensor::getMeasurementBufferSize() const
{
    int i = 0;
    while(logMessage[i] != 0x0) {
        i++;
    }
    return i; 
}

uint8_t LoggerSensor::configBlockSize( void ) const
{
    return 0;
}

bool LoggerSensor::isActive(void) const
{
   return true;
}

void LoggerSensor::setDefaultValues(void) 
{
}

void LoggerSensor::loadConfigValues(void) 
{
}

void LoggerSensor::writeConfigToBuffer( uint8_t* bufferBaseAddress) const 
{
}