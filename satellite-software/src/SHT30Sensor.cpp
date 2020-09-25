#include <Arduino.h>
#include <Wire.h>
#include "SHT30Sensor.h"
#include "Utilities.h"


#define SHT30_CMD_READ_SERIALNBR    0x3780  // read serial number
#define SHT30_CMD_READ_STATUS       0xF32D  // read status register
#define SHT30_CMD_CLEAR_STATUS      0x3041  // clear status register
#define SHT30_CMD_HEATER_ENABLE     0x306D  // enabled heater
#define SHT30_CMD_HEATER_DISABLE    0x3066  // disable heater
#define SHT30_CMD_SOFT_RESET        0x30A2  // soft reset
#define SHT30_CMD_MEAS_CLOCKSTR_H   0x2C06  // measurement: clock stretching, high repeatability
#define SHT30_CMD_MEAS_CLOCKSTR_M   0x2C0D  // measurement: clock stretching, medium repeatability
#define SHT30_CMD_MEAS_CLOCKSTR_L   0x2C10  // measurement: clock stretching, low repeatability
#define SHT30_CMD_MEAS_POLLING_H    0x2400  // measurement: polling, high repeatability
#define SHT30_CMD_MEAS_POLLING_M    0x240B  // measurement: polling, medium repeatability
#define SHT30_CMD_MEAS_POLLING_L    0x2416  // measurement: polling, low repeatability
#define SHT30_CMD_MEAS_PERI_05_H    0x2032  // measurement: periodic 0.5 mps, high repeatability
#define SHT30_CMD_MEAS_PERI_05_M    0x2024  // measurement: periodic 0.5 mps, medium repeatability
#define SHT30_CMD_MEAS_PERI_05_L    0x202F  // measurement: periodic 0.5 mps, low repeatability
#define SHT30_CMD_MEAS_PERI_1_H     0x2130  // measurement: periodic 1 mps, high repeatability
#define SHT30_CMD_MEAS_PERI_1_M     0x2126  // measurement: periodic 1 mps, medium repeatability
#define SHT30_CMD_MEAS_PERI_1_L     0x212D  // measurement: periodic 1 mps, low repeatability
#define SHT30_CMD_MEAS_PERI_2_H     0x2236  // measurement: periodic 2 mps, high repeatability
#define SHT30_CMD_MEAS_PERI_2_M     0x2220  // measurement: periodic 2 mps, medium repeatability
#define SHT30_CMD_MEAS_PERI_2_L     0x222B  // measurement: periodic 2 mps, low repeatability
#define SHT30_CMD_MEAS_PERI_4_H     0x2334  // measurement: periodic 4 mps, high repeatability
#define SHT30_CMD_MEAS_PERI_4_M     0x2322  // measurement: periodic 4 mps, medium repeatability
#define SHT30_CMD_MEAS_PERI_4_L     0x2329  // measurement: periodic 4 mps, low repeatability
#define SHT30_CMD_MEAS_PERI_10_H    0x2737  // measurement: periodic 10 mps, high repeatability
#define SHT30_CMD_MEAS_PERI_10_M    0x2721  // measurement: periodic 10 mps, medium repeatability
#define SHT30_CMD_MEAS_PERI_10_L    0x272A  // measurement: periodic 10 mps, low repeatability
#define SHT30_CMD_FETCH_DATA        0xE000  // readout measurements for periodic mode
#define SHT30_CMD_R_AL_LIM_LS       0xE102  // read alert limits, low set
#define SHT30_CMD_R_AL_LIM_LC       0xE109  // read alert limits, low clear
#define SHT30_CMD_R_AL_LIM_HS       0xE11F  // read alert limits, high set
#define SHT30_CMD_R_AL_LIM_HC       0xE114  // read alert limits, high clear
#define SHT30_CMD_W_AL_LIM_HS       0x611D  // write alert limits, high set
#define SHT30_CMD_W_AL_LIM_HC       0x6116  // write alert limits, high clear
#define SHT30_CMD_W_AL_LIM_LC       0x610B  // write alert limits, low clear
#define SHT30_CMD_W_AL_LIM_LS       0x6100  // write alert limits, low set
#define SHT30_CMD_NO_SLEEP          0x303E

#define SHT30_CRC_POLYNOMIAL        0x131   // P(x) = x^8 + x^5 + x^4 + 1 = 100110001

SHT30Sensor::SHT30Sensor(PersistedConfiguration& config)
    :   SensorBase(config)
{
    if (!begin())
    {
        Serial.println(F("ERROR: unable to initialize the SHT30"));
        setIsFound(false);
    } else {
        setIsFound(true);
    }
}

SHT30Sensor::~SHT30Sensor()
{

}

bool SHT30Sensor::sendCommand(uint16_t cmd, bool acceptNACKAtEnd )
{
    // when sending commands without clock stretching, the SHT30 will respond with a NACK rather than 
    // and ACK. This generates and error = 3 in the Wire library when ending transimission. The argument
    // acceptNACKAtEnd determines if the we expect an NACK in repsonse, and then tell SensorBase that getting a 
    // NACK on data transfer is OK.

    return writeData(i2cDeviceAddress(), (const uint8_t*)&cmd, 2, false, acceptNACKAtEnd);
}

uint16_t  SHT30Sensor::readStatus(void)
{
    uint8_t buffer[3];
    sendCommand(SHT30_CMD_READ_STATUS, true);
    if (!readData(buffer, 3, false)) {
        return 0xFFFF;
    }
    if (calculatCRC(buffer, 2, SHT30_CRC_POLYNOMIAL) != buffer[2]) {
        Serial.println(F("ERROR: CRC didn't match when readding SHT30 status."));
        return 0xFFFF;
    }
    return (uint16_t)buffer[0]*256 + buffer[1];
}

bool SHT30Sensor::begin(void)
{
    Wire.endTransmission();
    reset();

    uint_fast16_t status = readStatus();

    if (status == 0xFFFF) {
         return false;
    } 

    Serial.print(F("Found SHT30 with status = 0x"));
    Serial.print(status);
    Serial.print(F("\n"));
    return true;
}

void SHT30Sensor::reset(void)
{
    if (!sendCommand(SHT30_CMD_SOFT_RESET, true)) {
        Serial.println(F("ERROR: could not reset SHT30"));
        return;
    }
    delay(15);
}

void SHT30Sensor::setup(void)
{
    reset();

}
bool SHT30Sensor::isActive(void) const
{
//    return SensorBase::isActive();
    return false;
}

const uint8_t* SHT30Sensor::getCurrentMeasurementBuffer(void)
{

    return nullptr;
}