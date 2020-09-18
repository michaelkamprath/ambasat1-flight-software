#include "VoltageSensor.h"
#include "Utilities.h"

//
// Voltage Sensor
//
// Methodology for mesuring ATMEGA328P voltage was taken from:
//      https://www.sciencetronics.com/greenphotons/?p=1521
//

VoltageSensor::VoltageSensor(PersistedConfiguration& config)
  : SensorBase(config)
{

}

VoltageSensor::~VoltageSensor()
{

}

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
void VoltageSensor::setup(void)
{

}

int16_t VoltageSensor::readVccMilliVolts(void) const
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
VoltageSensor::getCurrentMeasurementBuffer(void)
{
    // put results in buffer and return
    hton_int16(this->readVccMilliVolts(), _buffer);
    return _buffer;
}