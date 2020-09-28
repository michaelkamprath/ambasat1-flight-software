#include <Arduino.h>
#include "Si1132Sensor.h"
#include "Logging.h"

//
// TODO:
//  - Load calibration data from device and apply for more accurate measurements
//  - Create sensitivity configurations to allow for easy adjustments under various conditions.
//

#define SI1132_PART_ID_REG          0x00
#define SI1132_REV_ID_REG           0x01
#define SI1132_SEQ_ID_REG           0x02
#define SI1132_INT_CFG_REG          0x03
#define SI1132_IRQ_ENABLE_REG       0x04
#define SI1132_HW_KEY_REG           0x07
#define SI1132_MEAS_RATE0_REG       0x08
#define SI1132_MEAS_RATE1_REG       0x09
#define SI1132_UCOEF0_REG           0x13
#define SI1132_UCOEF1_REG           0x14
#define SI1132_UCOEF2_REG           0x15
#define SI1132_UCOEF3_REG           0x16
#define SI1132_PARAM_WR_REG         0x17
#define SI1132_COMMAND_REG          0x18
#define SI1132_RESPONSE_REG         0x20
#define SI1132_IRQ_STATUS_REG       0x21
#define SI1132_ALS_VIS_DATA0_REG    0x22
#define SI1132_ALS_VIS_DATA1_REG    0x23
#define SI1132_ALS_IR_DATA0_REG     0x24
#define SI1132_ALS_IR_DATA1_REG     0x25
#define SI1132_AUX_DATA0_REG        0x2C
#define SI1132_AUX_DATA1_REG        0x2D
#define SI1132_PARAM_RD_REG         0x2E
#define SI1132_CHIP_STAT_REG        0x30

#define SI1132_CMD_RESET        0x01
#define SI1132_CMD_ALS_FORCE    0x06
#define SI1132_CMD_ALS_AUTO     0x0E
#define SI1132_CMD_PARAM_QUERY  0x80
#define SI1132_CMD_PARAM_SET    0xA0

#define SI1132_PARAM_CHLIST                 0x01
#define SI1132_PARAM_ALS_IR_ADCMUX          0x0E
#define SI1132_PARAM_ALS_VIS_ADC_COUNTER    0x10
#define SI1132_PARAM_ALS_VIS_ADC_GAIN       0x11
#define SI1132_PARAM_ALS_VIS_ADC_MISC       0x12
#define SI1132_PARAM_ALS_IR_ADC_COUNTER     0x1D
#define SI1132_PARAM_ALS_IR_ADC_GAIN        0x1E
#define SI1132_PARAM_ALS_IR_ADC_MISC        0x1F
#define SI1132_PARAM_

Si1132Sensor::Si1132Sensor(PersistedConfiguration& config)
    :   SensorBase(config),
        _MEAS_RATE0(SI1132_MEAS_RATE0_REG),
        _MEAS_RATE1(SI1132_MEAS_RATE1_REG)
{

    if (!begin())
    {
        PRINTLN_ERROR(F("ERROR: Unable to initialize the Si1132"));
        setIsFound(false);
    } else {
        setIsFound(true);
    }
}

Si1132Sensor::~Si1132Sensor()
{

}
void Si1132Sensor::reset(void)
{
    if (!isFound()) {
        return;
    }

    writeRegister(SI1132_MEAS_RATE0_REG, 0x00);
    writeRegister(SI1132_MEAS_RATE1_REG, 0x00);
    writeRegister(SI1132_IRQ_ENABLE_REG, 0x00);
    writeRegister(SI1132_INT_CFG_REG, 0x00);
    writeRegister(SI1132_IRQ_STATUS_REG, 0xFF);

    writeRegister(SI1132_COMMAND_REG, SI1132_CMD_RESET);
    delay(20);
    writeRegister(SI1132_HW_KEY_REG, 0x17);
    delay(20);
}

bool Si1132Sensor::begin(void)
{
    uint8_t device_id = readRegister(SI1132_PART_ID_REG);

    if (device_id != 0x32) {
        PRINT_ERROR(F("ERROR when initializing Si1132: device_id = 0x"));
        PRINT_HEX_ERROR(device_id);
        PRINT_ERROR(F("\n"));
        return false;
    }
    uint8_t revision_id = readRegister(SI1132_REV_ID_REG);
    uint8_t sequence_id = readRegister(SI1132_SEQ_ID_REG);
    PRINT_INFO(F("Found Si1132 UV Light Sensor with revision ID = "));
    PRINT_INFO(revision_id);
    PRINT_INFO(F(", sequence ID = "));
    PRINT_INFO(sequence_id);
    PRINT_INFO(F("\n"));

    if (sequence_id == 0x01) {
        // deal with a device-level software bug in sequence=0x01 models as documented in spec
        _MEAS_RATE0 = 0x0A;
        _MEAS_RATE1 = 0x08;
    }
    return true;
}

bool Si1132Sensor::isActive(void) const
{
    return SensorBase::isActive();
}

void Si1132Sensor::setup(void)
{
    if (!isFound()) {
        return;
    }

    reset();
 
    // sendCommand(SI1132_CMD_RESET);
    // delay(10);

    // set UCOEF[0:3] to the default values
    writeRegister(SI1132_UCOEF0_REG, 0x7B);
    writeRegister(SI1132_UCOEF1_REG, 0x6B);
    writeRegister(SI1132_UCOEF2_REG, 0x01);
    writeRegister(SI1132_UCOEF3_REG, 0x00);
 
    // turn on UV Index, ALS IR, and ALS Visible
    setParameter(SI1132_PARAM_CHLIST, 0xB0);
        
    // set up VIS sensor
    //  clock divide = 1
    setParameter(SI1132_PARAM_ALS_VIS_ADC_GAIN, 0x00);
    //  ADC count at 511
    setParameter(SI1132_PARAM_ALS_VIS_ADC_COUNTER, 0b01110000);
    // set for high sifnal (e.g., bright sun)
    // param_res = setParameter(SI1132_PARAM_ALS_VIS_ADC_MISC, 0b00100000);
 
    // set up IR sensor
    //  clock divide = 1
    setParameter(SI1132_PARAM_ALS_IR_ADC_GAIN, 0x00);
   //  ADS count at 511
    setParameter(SI1132_PARAM_ALS_IR_ADC_COUNTER, 0b01110000);
    //  small IR photodiode
    setParameter(SI1132_PARAM_ALS_IR_ADCMUX, 0x00);
    //  set IR_RANGE bit for high signal. Must do "read and modify" per spec
    // uint8_t cur_value = readParameter(SI1132_PARAM_ALS_IR_ADC_MISC)
    // param_res = setParameter(SI1132_PARAM_ALS_IR_ADC_MISC, 0b00100000|cur_value);

    // Place ins forced measurement mode
    writeRegister(_MEAS_RATE0, 0x00);
    writeRegister(_MEAS_RATE1, 0x00);

}

#define LOOP_TIMEOUT_MS 200
bool Si1132Sensor::waitUntilSleep(void) {

    for (int16_t i = 0; i < LOOP_TIMEOUT_MS; i++ ) {
        uint8_t val = readRegister(SI1132_CHIP_STAT_REG);
        if (val == 0b00000001) {
            // chip is asleep
            return true;
        }
        delay(1);
    }
    return false;
}

uint8_t Si1132Sensor::readResponseRegister(void)
{
    return readRegister(SI1132_RESPONSE_REG);
}

bool Si1132Sensor::sendCommand(uint8_t cmd_value)
{
    writeRegister(SI1132_COMMAND_REG, 0x00);
    uint8_t response = readResponseRegister();
    while (response != 0x00) {
        writeRegister(SI1132_COMMAND_REG, 0x00);
        delay(5);
        response = readResponseRegister();
    }
    if (!writeRegister(SI1132_COMMAND_REG, cmd_value)) {
        PRINTLN_ERROR(F("ERROR writing to command Si1132 register"));
        return false;
    }
    int16_t counter = 0;
    response = readResponseRegister();
    while ((counter < 10) && (response == 0x00)) {
        writeRegister(SI1132_COMMAND_REG, cmd_value);
        delay(5);
        response = readResponseRegister();
        counter++;
    }
    return (response != 0x00);
}

bool Si1132Sensor::setParameter(uint8_t param, uint8_t value)
{
    uint8_t cmd = SI1132_CMD_PARAM_SET | param;
    // set PARAM_WR register
    writeRegister(SI1132_PARAM_WR_REG, value);
    // send command
    bool success = sendCommand(cmd);
   
    if (!success) {
        // read parameter
        sendCommand(SI1132_CMD_PARAM_QUERY|param);
        uint8_t paramValue = readRegister(SI1132_PARAM_RD_REG);
        PRINT_ERROR(F("ERROR sending param write command for Si1132 parameter = 0x"));
        PRINT_HEX_ERROR(param);
        PRINT_DEBUG(F(", final parameter value = "));
        PRINT_HEX_DEBUG(paramValue);
        PRINT_DEBUG(F(", desired parameter value = "));
        PRINT_HEX_DEBUG(value);
        PRINT_ERROR(F("\n"));
        return 0;
    }
    // read results in PARAM_RD
    uint8_t param_res = readRegister(SI1132_PARAM_RD_REG);
    bool result = (param_res == value);
    if (!result) {
        PRINT_ERROR(F("ERROR setting Si1132 parameter 0x"));
        PRINT_HEX_ERROR(param);
        PRINT_DEBUG(F(" to 0x"));
        PRINT_HEX_DEBUG(value);
        PRINT_DEBUG(F(" but got 0x"));
        PRINT_HEX_DEBUG(param_res);
        PRINT_ERROR(F("\n"));
    }
    return result;
}

uint8_t Si1132Sensor::readParameter(uint8_t param)
{
    uint8_t cmd = SI1132_CMD_PARAM_QUERY | param;

    bool success = sendCommand(cmd);
    if (!success) {
        // read parameter
        PRINT_ERROR(F("ERROR reading Si1132 parameter 0x"));
        PRINT_HEX_ERROR(param);
        PRINT_ERROR(F("\n"));
        return 0;
    }
    // read results in PARAM_RD
    return readRegister(SI1132_PARAM_RD_REG);
}

const uint8_t* Si1132Sensor::getCurrentMeasurementBuffer(void)
{
    if (!isActive()) {
        return nullptr;
    }

    // start the measurements
    sendCommand(SI1132_CMD_ALS_FORCE);

    // wait for measurements
    delay(1000);
    
    // Note that network order is big endian. Arrange bytes accordingly.
    uint8_t localBuffer[4];

    // UV
    if (!readRegisters(SI1132_AUX_DATA0_REG, localBuffer, 2 )) {
        PRINTLN_ERROR("ERROR reading UV data on Si1132");
        return nullptr;
    }
    _buffer[0] = localBuffer[1];
    _buffer[1] = localBuffer[0];

    // vissible and IR
    if (!readRegisters(SI1132_ALS_VIS_DATA0_REG, localBuffer, 4 )) {
        PRINTLN_ERROR("ERROR reading vis and ir data on Si1132");
        return nullptr;
    }
    _buffer[2] = localBuffer[1];
    _buffer[3] = localBuffer[0];
    _buffer[4] = localBuffer[3];
    _buffer[5] = localBuffer[2];
  
    PRINT_DEBUG(F("    UV index = "));
    PRINT_DEBUG(_buffer[0]*256+_buffer[1]);
    PRINT_DEBUG(F(", visible = "));
    PRINT_DEBUG(_buffer[2]*256+_buffer[3]);
    PRINT_DEBUG(F(", IR = "));
    PRINT_DEBUG(_buffer[4]*256+_buffer[5]);
    PRINT_DEBUG(F("\n"));
    return _buffer;
}
