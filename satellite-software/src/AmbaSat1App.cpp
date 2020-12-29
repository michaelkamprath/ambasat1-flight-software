#include "AmbaSat1App.h"

#include <hal/hal.h>
#include <LowPower.h>
#include "Utilities.h"
#include "Logging.h"

//
// Satellite Physical Setup
//

#define LED_PIN 9

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = LMIC_UNUSED_PIN,
  .dio = {2, A2, LMIC_UNUSED_PIN},
};


//
// Global Variable so LMIC functions can access app object
//

AmbaSat1App* AmbaSat1App::gApp = nullptr;

//
// Useful #defines
//

#define SENSOR_STATUS_LSM9DS1_FOUND     0b00000001
#define SENSOR_STATUS_LSM9DS1_ACTIVE    0b00000010
#define SENSOR_STATUS_MISSION_FOUND     0b00010000
#define SENSOR_STATUS_MISSION_ACTIVE    0b00100000

//
// AmbaSat1App
//

AmbaSat1App::AmbaSat1App()
    :   _config(),
        _lsm9DS1Sensor(_config),
        _missionSensor(_config),
        _sleeping(false)
#ifdef ENABLE_AMBASAT_COMMANDS
        ,
        _queuedCommandPort(0xFF)
#endif
{
    if (AmbaSat1App::gApp != nullptr) {
        // complain loudly. Only one app object should be created.
        PRINT_ERROR(F("ERROR - More than one AmbaSat1App object created."));
    }
    else {
        AmbaSat1App::gApp = this;
    }
    _config.setSensorConfigDelegates(&_lsm9DS1Sensor, &_missionSensor);
}

AmbaSat1App::~AmbaSat1App()
{
    AmbaSat1App::gApp = nullptr;
}

void AmbaSat1App::setup()
{
    // Turn on LED during setup
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    digitalWrite(LED_PIN, HIGH);

    _config.init();
    _lsm9DS1Sensor.setup();
    _missionSensor.setup();

    _config.setUplinkSleepCycles(DEFAULT_RATE_VALUE, true);

    //
    // Set up LoRaWAN radio
    //

    os_init();
    LMIC_reset();
    LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);
//    LMIC_setDrTxpow(DR_SF9, KEEP_TXPOW);

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));

    // network ID 0x01 = Expiremental
    // network ID 0x13 = The Things Network

    LMIC_setSession (0x13, DEVADDR, nwkskey, appskey);

#if defined(CFG_eu868)
    PRINTLN_INFO(F("Transmitting using eu868 frequency plan"));
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    // NA-US channels 0-71 are configured automatically
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
#elif defined(CFG_us915)
    PRINTLN_INFO(F("Transmitting using us915 frequency plan"));
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
#endif
    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7, 14);

    //
    // set the LMIC uplink frame count
    //
    LMIC.seqnoUp = _config.getUplinkFrameCount();

    //
    // Finished Setting up. Tun LED off
    //
    digitalWrite(LED_PIN, LOW);
}

void AmbaSat1App::loop() 
{
    PRINTLN_INFO(F("Transmitting Satellite Status."));
    sendSensorPayload(*this);
    _sleeping = false;

    if (_lsm9DS1Sensor.isActive()) {
        PRINTLN_INFO(F("Transmitting LSM9DS1 sensor."));
        sendSensorPayload(_lsm9DS1Sensor);
        _sleeping = false;
    }

    if (_missionSensor.isActive()) {
        PRINTLN_INFO(F("Transmitting mission sensor"));
        sendSensorPayload(_missionSensor);
        _sleeping = false;
    }
    //
    // technically there is some risk that the satellite will loose power between
    // the first transmission above and the last one, and in such case we will not
    // capture the uplink frame count. We are accepting that risk in order to reduce
    // number of time we write to EEPROM.
    //
    _config.setUplinkFrameCount(LMIC.seqnoUp);
    _config.setUplinkPattern(MISSION_SENSOR_PAYLOAD);

    // flush serial before going to sleep
    Serial.flush();

    // sleep device for designated sleep cycles
    for (int i=0; i < _config.getUplinkSleepCycles(); i++)
    {
        LowPower.powerDown(TIME_INTERVAL, ADC_OFF, BOD_OFF);    //sleep TIME_INTERVAL seconds * sleepcycles
    }   
}

void AmbaSat1App::sendSensorPayload(LoRaPayloadBase& sensor)
{
    // wait for any in process transmission to end. This really shouldn't happen, but can 
    // if the prior transmission received an downlink requiring an ACK.
    if ((LMIC.opmode&0x00FF) != OP_NONE) {
        PRINTLN_INFO(F("  Waiting on prior transmission ..."));
        while((LMIC.opmode&0x00FF) != OP_NONE) {
            os_runloop_once();
        }
        PRINTLN_INFO(F("    Complete."));
        // delay a little for radio to reset
        delay(2000);
    }

    const uint8_t* data_ptr = sensor.getCurrentMeasurementBuffer();

    if (data_ptr == nullptr) {
        PRINTLN_INFO(F("  Sensor data is NULL."));
        return;
    }
#if LOG_LEVEL >= LOG_LEVEL_INFO
    PRINT_INFO(F("  Sending payload = "));
    print_buffer(data_ptr, sensor.getMeasurementBufferSize());
    Serial.flush();
#endif  

    // LMIC seems to crash here if we previously just recieved a downlink AND
    // there are any pending data in the Serial queue. So flush the Serial queue.
    Serial.flush();
    LMIC_setTxData2(
        sensor.getPort(),
        (xref2u1_t)data_ptr,
        sensor.getMeasurementBufferSize(),
        0
    ); 
  
    // Again, LMIC seems to flake out after recived downlinks without this delay. 
    // Seems to be some interaction with the serial prints that follow. Since 
    // LMIC is a black box and 50 milliseconds isn't all that much, so just wait.
    delay(50);

    // Must wait for the TX operations of LMIC.opmode to go to zero in order to handle
    // any downlink ACK that was requested. 
    // TODO: determine the utility of the _sleeping variable given the forced wait until the radio is complete.
    while((!_sleeping) || (LMIC.opmode&0x00FF) != OP_NONE) {
        os_runloop_once();
    }

    #ifdef ENABLE_AMBASAT_COMMANDS
    // handle any command that got queued
    processQueuedCommand();
    #endif
}

#ifdef ENABLE_AMBASAT_COMMANDS
void AmbaSat1App::queueCommand(uint8_t port, uint8_t* recievedData, uint8_t recievedDataLen)
{
    if ((_queuedCommandPort == 0xFF) && (recievedDataLen <= QUEUED_COMMAND_BUFFER_SIZE )) {
        _queuedCommandPort = port;
        memcpy(_queuedCommandDataBuffer, recievedData, recievedDataLen);
        _queuedCommandDataLength = recievedDataLen;
        PRINT_DEBUG(F("  queued command on port "));
        PRINT_DEBUG(port);
        PRINT_DEBUG(F(" with data len = "));
        PRINT_DEBUG(recievedDataLen);
        PRINT_DEBUG(F("\n"));
    }
}

void AmbaSat1App::processQueuedCommand(void)
{
    if (_queuedCommandPort == 0xFF) {
        PRINTLN_DEBUG(F("No queued commands to process"));
        return;
    }

    // this method decodes the command header and routes the data to the appropriate handler
    PRINT_DEBUG(F("Recieved command on port "));
    PRINT_DEBUG(_queuedCommandPort);
    PRINT_DEBUG(F(", payload = "));
#if LOG_LEVEL >= LOG_LEVEL_DEBUG
    print_buffer(_queuedCommandDataBuffer, _queuedCommandDataLength);
#endif

    // The port is used to determine which handler the command should be routed to:
    //      port 2 = satellite commands
    //      port 3 = LSM9DS1 commands
    //      port 4 = Mission sensor comands

    // The command payload is:
    //      uint16_t - command sequence ID: ground software uses this value to identify the command sent
    //      uint8_t  - command: the command ID, to be interpreted by the command handler.
    //      uint8_t* - comannd data: a variable length of data that provides parameters to the command.
    //                 The size of this data blob is determined by the command.
    //

    uint16_t cmdSeuqenceID = (uint16_t)ntoh_int16(&_queuedCommandDataBuffer[0]);
    uint8_t cmd = _queuedCommandDataBuffer[2];
    uint8_t* cmdData = _queuedCommandDataLength > 3 ? &_queuedCommandDataBuffer[3] : nullptr;
    uint8_t cmdDataSize = _queuedCommandDataLength >= 3 ? (_queuedCommandDataLength-3) : 0;

    uint8_t status = CMD_STATUS_UNIMPLEMENTED;
    switch (_queuedCommandPort) {
        case 2:
            status = this->handleCommand(cmdSeuqenceID, cmd, cmdData, cmdDataSize);
            break;
        case 3:
            status = _lsm9DS1Sensor.handleCommand(cmdSeuqenceID, cmd, cmdData, cmdDataSize);
            break;
        case 4:
            status = _missionSensor.handleCommand(cmdSeuqenceID, cmd, cmdData, cmdDataSize);
            break;
        default:
            PRINT_ERROR(F("ERROR - received cmd on port = "));
            PRINT_ERROR(_queuedCommandPort);
            PRINT_ERROR(F("\n"));
            break;
    }

    // reset port back to no command
    _queuedCommandPort = 0xFF;

    // responde with command status
    uint8_t replyBuffer[3];
    hton_int16(cmdSeuqenceID, &replyBuffer[0]);
    replyBuffer[2] = status;

#if LOG_LEVEL >= LOG_LEVEL_INFO
    PRINT_INFO(F("  Sending command response payload = "));
    print_buffer(replyBuffer, 3);
#endif 

    // LMIC seems to crash here if we previously just recieved a downlink AND
    // there are any pending data in the Serial queue. So flush the Serial queue.
    Serial.flush();

    _sleeping = false;
    LMIC_setTxData2(
        PORT_CMD_STATUS,
        (xref2u1_t)replyBuffer,
        3,
        0
    ); 
  
    // Again, LMIC seems to flake out after recived downlinks without this delay. 
    // Seems to be some interaction with the serial prints that follow. Since 
    // LMIC is a black box and 50 milliseconds isn't all that much, so just wait.
    delay(50);

    // Must wait for the TX operations of LMIC.opmode to go to zero in order to handle
    // any downlink ACK that was requested. 
    // TODO: determine the utility of the _sleeping variable given the forced wait until the radio is complete.
    while((!_sleeping) || (LMIC.opmode&0x00FF) != OP_NONE) {
        os_runloop_once();
    }
}

uint8_t AmbaSat1App::handleCommand(uint16_t cmdSequenceID, uint8_t command, uint8_t* recievedData, uint8_t recievedDataLen)
{
    // Commands are identified in the first byte. Commands that the satellite supports:
    //
    //  0x01 - Blink LED. The second byte is is split into two nibbles. The uppper 2 bits indicate the
    //         blink period as follows:
    //              00 = 0.1 second blinks
    //              01 = 0.5 second blinks
    //              10 = 1 second blinks
    //              11 = 2 second blinks.
    //         The lower 6 bits are used to indicate the number of blinks.
    //

    if (command == 0x01) {
        return executeBlinkCmd(recievedData, recievedDataLen);
    } else if (command == 0x02) {
        return executeUplinkPatternCmd(recievedData, recievedDataLen);
    } else if (command == 0x03) {
        return executeUplinkRateCmd(recievedData, recievedDataLen);
    } else if (command == 0x04) {
        return executeSetFrameCountCmd(recievedData, recievedDataLen);
    }

    return CMD_STATUS_UNKNOWN_CMD;
}

uint8_t AmbaSat1App::executeBlinkCmd(uint8_t* recievedData, uint8_t recievedDataLen) 
{
    if (recievedDataLen != 1 ) {
        return CMD_STATUS_BAD_DATA_LEN;
    }

    uint8_t blinkCount = recievedData[0]&0x3F;
    uint16_t blinkDurationMillis = 100;
    switch (recievedData[0]&0xC0) {
        default:
        case 0x00:
            blinkDurationMillis = 100;
            break;
        case 0x40:
            blinkDurationMillis = 500;
            break;                   
        case 0x80:
            blinkDurationMillis = 1000;
            break;                   
        case 0xC0:
            blinkDurationMillis = 2000;
            break;                   
    }
    PRINT_DEBUG(F("  blink command. count = "));
    PRINT_DEBUG(blinkCount);
    PRINT_DEBUG(F(", duration = "));
    PRINT_DEBUG(blinkDurationMillis);   
    PRINT_DEBUG(F(" ms\n"));

    for (int16_t i = 0; i <blinkCount; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(blinkDurationMillis);
        digitalWrite(LED_PIN, LOW);
        if (i < blinkCount-1) {
            delay(blinkDurationMillis);
        }
    }
    delay(100);
    return CMD_STATUS_SUCCESS;   
}

uint8_t AmbaSat1App::executeUplinkPatternCmd(uint8_t* recievedData, uint8_t recievedDataLen)
{
    if (recievedDataLen != 1 ) {
        return CMD_STATUS_BAD_DATA_LEN;
    }
    UplinkPayloadType pattern = static_cast<UplinkPayloadType>(recievedData[0]);
    _config.setUplinkPattern(pattern, true);

    PRINT_DEBUG(F("  set the uplink pattern to "));
    PRINT_DEBUG(pattern);
    PRINT_DEBUG(F("\n"));
    return CMD_STATUS_UNIMPLEMENTED;
}

uint8_t AmbaSat1App::executeUplinkRateCmd(uint8_t* recievedData, uint8_t recievedDataLen)
{
    if (recievedDataLen != 1 ) {
        return CMD_STATUS_BAD_DATA_LEN;
    }

    uint8_t rateValue = recievedData[0];
    _config.setUplinkSleepCycles(rateValue, true);
    PRINT_DEBUG(F("  set the uplink rate to "));
    PRINT_DEBUG(rateValue);
    PRINT_DEBUG(F(" sleep cycles\n"));
    return CMD_STATUS_SUCCESS;
}

uint8_t AmbaSat1App::executeSetFrameCountCmd(uint8_t* recievedData, uint8_t recievedDataLen)
{
    if (recievedDataLen != 2 ) {
        return CMD_STATUS_BAD_DATA_LEN;
    }

    uint16_t frameCount = (uint16_t)ntoh_int16(recievedData);
    _config.setUplinkFrameCount(frameCount, true);
    LMIC.seqnoUp = frameCount;

    PRINT_DEBUG(F("  reset the uplink frame count to "));
    PRINT_DEBUG(frameCount);
    PRINT_DEBUG(F("\n"));

    return CMD_STATUS_SUCCESS;
}


#endif
// initial job
static osjob_t initjob;

void initfunc(osjob_t* j)
{
    // reset MAC state
    LMIC_reset();
    LMIC_setClockError(MAX_CLOCK_ERROR * 5 / 100);

    // init done - onEvent() callback will be invoked...
    PRINTLN_INFO(F("LMIC initFunc complete"));
}

// These callbacks are only used in over-the-air activation, so they are
// left empty here (cannot be left out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

// =========================================================================================================================================
// onEvent
// =========================================================================================================================================
void onEvent(ev_t ev)
{

    switch (ev)
    {
        case EV_SCAN_TIMEOUT:
            PRINTLN_DEBUG(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            PRINTLN_DEBUG(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            PRINTLN_DEBUG(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            PRINTLN_DEBUG(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            PRINTLN_DEBUG(F("EV_JOINING"));
            break;
        case EV_JOINED:
            PRINTLN_DEBUG(F("EV_JOINED"));
            // Disable link check validation (automatically enabled
            // during join, but not supported by TTN at this time).
            LMIC_setLinkCheckMode(0);
            digitalWrite(LED_PIN, LOW);
            break;
        case EV_RFU1:
            PRINTLN_DEBUG(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            PRINTLN_DEBUG(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            PRINTLN_DEBUG(F("EV_REJOIN_FAILED"));
            // Re-init
            os_setCallback(&initjob, initfunc);
            break;
        case EV_TXCOMPLETE:
            AmbaSat1App::gApp->_sleeping = true;
            if (LMIC.dataLen > 0)
            {
#ifdef ENABLE_AMBASAT_COMMANDS
                AmbaSat1App::gApp->queueCommand(
                    LMIC.frame[LMIC.dataBeg-1],
                    &LMIC.frame[LMIC.dataBeg],
                    LMIC.dataLen
                );

#else
                PRINTLN_DEBUG(F("WARNING Recieved a downlink but code is not enabled to process it."));
#endif
            }
            PRINTLN_ERROR(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            Serial.flush();
            break;
        case EV_LOST_TSYNC:
            PRINTLN_DEBUG(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            PRINTLN_DEBUG(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            PRINTLN_DEBUG(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            PRINTLN_DEBUG(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            PRINTLN_DEBUG(F("EV_LINK_ALIVE"));
            break;
        default:
            PRINTLN_DEBUG(F("Unknown event"));
            break;
    }
}


//
// Satellite Status Payload Handling
//

//
// Two different methods for calculating voltage.
//
#if 0
#define ADMUX_ADCMASK  ((1 << MUX3)|(1 << MUX2)|(1 << MUX1)|(1 << MUX0))
#define ADMUX_REFMASK  ((1 << REFS1)|(1 << REFS0))

#define ADMUX_REF_AREF ((0 << REFS1)|(0 << REFS0))
#define ADMUX_REF_AVCC ((0 << REFS1)|(1 << REFS0))
#define ADMUX_REF_RESV ((1 << REFS1)|(0 << REFS0))
#define ADMUX_REF_VBG  ((1 << REFS1)|(1 << REFS0))

#define ADMUX_ADC_VBG  ((1 << MUX3)|(1 << MUX2)|(1 << MUX1)|(0 << MUX0))

void VoltageSensor::setup(void)
{
  // initialize the ADC
  ADMUX = (1 << REFS1) | (1 << REFS0)
        | (0 << ADLAR)
        | (0 << MUX3) | (0 << MUX2) | (0 << MUX1) | (0 << MUX0);
  ADCSRA = (1 << ADEN)
         | (0 << ADSC)
         | (0 << ADATE)
         | (1 << ADPS2)|(0 << ADPS1)|(1 << ADPS0);  
  ADCSRA |= (1 << ADSC);         // start dummy conversion
  while (ADCSRA & (1 << ADSC));  // wait for dummy to finish
}

int16_t VoltageSensor::readVccMilliVolts(void) const
{
    ADMUX &= ~(ADMUX_REFMASK | ADMUX_ADCMASK);
    ADMUX |= ADMUX_REF_AVCC;      // select AVCC as reference
    ADMUX |= ADMUX_ADC_VBG;       // measure bandgap reference voltage

    _delay_us(500);               // a delay rather than a dummy measurement is needed to give a stable reading!
    ADCSRA |= (1 << ADSC);        // start conversion
    while (ADCSRA & (1 << ADSC)); // wait to finish

    int16_t volts = (1100UL*1023/ADC);     // AVcc = Vbg/ADC*1023 = 1.1V*1023/ADC 
    return volts;
}
#else

int16_t AmbaSat1App::readVccMilliVolts(void) const
{
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif

  delay(10); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  int32_t result = (high<<8) | low;

  result = 1125300L / result;
  return result < 32767 ? result : 32767; // Vcc in millivolts
}
#endif

const uint8_t* 
AmbaSat1App::getCurrentMeasurementBuffer(void)
{
    // Buffer format
    //
    //      uint32_t    reboot count
    //      uint16_t    voltage
    //      uint8_t     sensor status
    //
    //  TOTAL BUFFER SIZE --> 7 bytes
    //

    hton_int32(_config.getRebootCount(), &(_buffer[0]));
    hton_int16(this->readVccMilliVolts(), &(_buffer[4]));

    // calculate the sensor status byte
    uint8_t sensorStatus = 0x00;
    if (_lsm9DS1Sensor.isFound()) {
        sensorStatus |= SENSOR_STATUS_LSM9DS1_FOUND;
    }
    if (_lsm9DS1Sensor.isActive()) {
        sensorStatus |= SENSOR_STATUS_LSM9DS1_ACTIVE;
    }
    if (_missionSensor.isFound()) {
        sensorStatus |= SENSOR_STATUS_MISSION_FOUND;
    }
    if (_missionSensor.isActive()) {
        sensorStatus |= SENSOR_STATUS_MISSION_ACTIVE;
    }

    _buffer[6] = sensorStatus;
    
    return _buffer;
}