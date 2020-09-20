#include "AmbaSat1App.h"

#include <hal/hal.h>
#include <LowPower.h>
#include "Utilities.h"

//
// Satellite Physical Setup
//

#define LED_PIN 9
#define SLEEPCYCLES 75

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 7,
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
{
    if (AmbaSat1App::gApp != nullptr) {
        // complain loudly. Only one app object should be created.
        Serial.println(F("ERROR - More than one AmbaSat1App object created."));
    }
    else {
        AmbaSat1App::gApp = this;
    }
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

    _lsm9DS1Sensor.setup();
    _missionSensor.setup();

    //
    // Set up LoRaWAN radio
    //

    os_init();
    LMIC_reset();
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
    // Finished Setting up. Tun LED off
    //
    digitalWrite(LED_PIN, LOW);

    //
    // set the LMIC uplink frame count
    //
    LMIC.seqnoUp = _config.getUplinkFrameCount();

}

void AmbaSat1App::loop() 
{
    digitalWrite(LED_PIN, HIGH);
    Serial.println(F("Transmitting Satellite Status."));
    sendSensorPayload(*this);
    while(!_sleeping) {
        os_runloop_once();
    }
    _sleeping = false;

    if (_lsm9DS1Sensor.isActive()) {
        Serial.println(F("Transmitting LSM9DS1 sensor."));
        sendSensorPayload(_lsm9DS1Sensor);
        while(!_sleeping) {
            os_runloop_once();
        }
        _sleeping = false;
    }

    if (_missionSensor.isActive()) {
        Serial.println(F("Transmitting Mission sensor."));
        sendSensorPayload(_missionSensor);
        while(!_sleeping) {
            os_runloop_once();
        }
        _sleeping = false;
    }
    //
    // technically there is some risk that the satellite will loose power between
    // the first transmission above and the last one, and in such case we will not
    // capture the uplink frame count. We are accepting that risk in order to reduce
    // number of time we write to EEPROM.
    //
    _config.setUplinkFrameCount(LMIC.seqnoUp);
    digitalWrite(LED_PIN, LOW);
    for (int i=0; i < SLEEPCYCLES; i++)
    {
        LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);    //sleep 8 seconds * sleepcycles
    }   
}

void AmbaSat1App::sendSensorPayload(LoRaPayloadBase& sensor)
{
    // wait for any in process transmission to end
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("Current OP_TXRXPEND is running so skipping transmission"));
        return;
    }

    const uint8_t* data_ptr = sensor.getCurrentMeasurementBuffer();

    if (data_ptr == nullptr) {
        Serial.println(F("Sensor data is NULL."));
        return;
    }
    LMIC_setTxData2(
        sensor.getPort(),
        (xref2u1_t)data_ptr,
        sensor.getMeasurementBufferSize(),
        0
    ); 
    Serial.print(F("    Sending payload = { "));
    for (uint8_t i = 0; i < sensor.getMeasurementBufferSize(); i++ ) {
        Serial.print(F("0x"));
        Serial.print( data_ptr[i], HEX);
        if (i < sensor.getMeasurementBufferSize() - 1 ) {
            Serial.print(F(", "));
        }
    }
    Serial.print(F(" }\n"));
}

// initial job
static osjob_t initjob;

void initfunc(osjob_t* j)
{
    // reset MAC state
    LMIC_reset();

    // init done - onEvent() callback will be invoked...
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
    int i, j;

    switch (ev)
    {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
        break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
        break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
        break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
        break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
        break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            // Disable link check validation (automatically enabled
            // during join, but not supported by TTN at this time).
            LMIC_setLinkCheckMode(0);
            digitalWrite(LED_PIN, LOW);
        break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
        break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
        break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            // Re-init
            os_setCallback(&initjob, initfunc);
        break;
        case EV_TXCOMPLETE:
            AmbaSat1App::gApp->_sleeping = true;

            if (LMIC.dataLen)
            {
                // data received in rx slot after tx
                // if any data received, a LED will blink
                // this number of times, with a maximum of 10
                Serial.print(F("Data Received: "));
                Serial.println(LMIC.frame[LMIC.dataBeg], HEX);
                i=(LMIC.frame[LMIC.dataBeg]);
                // i (0..255) can be used as data for any other application
                // like controlling a relay, showing a display message etc.
                if (i>10)
                {
                    i=10;     // maximum number of BLINKs
                }

                for(j = 0; j < i ; j++)
                {
                    digitalWrite(LED_PIN, HIGH);
                    delay(200);
                    digitalWrite(LED_PIN, LOW);
                    delay(400);
                }
            }

            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            delay(50);  // delay to complete Serial Output before Sleeping

            // Schedule next transmission
            // next transmission will take place after next wake-up cycle in main loop
        break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
        break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
        break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
        break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
        break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
        break;
        default:
            Serial.println(F("Unknown event"));
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