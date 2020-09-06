#include "VoltageSensor.h"
#include "Utilities.h"

//
// Voltage Sensor
//
// Methodology for mesuring ATMEGA328P voltage was taken from:
//      https://www.sciencetronics.com/greenphotons/?p=1521
//

VoltageSensor::VoltageSensor()
{

}

VoltageSensor::~VoltageSensor()
{

}

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

const uint8_t* 
VoltageSensor::getCurrentMeasurementBuffer(void)
{
    ADMUX &= ~(ADMUX_REFMASK | ADMUX_ADCMASK);
    ADMUX |= ADMUX_REF_AVCC;      // select AVCC as reference
    ADMUX |= ADMUX_ADC_VBG;       // measure bandgap reference voltage

    _delay_us(500);               // a delay rather than a dummy measurement is needed to give a stable reading!
    ADCSRA |= (1 << ADSC);        // start conversion
    while (ADCSRA & (1 << ADSC)); // wait to finish

    int16_t volts = (1100UL*1023/ADC);     // AVcc = Vbg/ADC*1023 = 1.1V*1023/ADC 

    // put results in buffer and return
    hton_int16(volts, _buffer);
    return _buffer;
}
