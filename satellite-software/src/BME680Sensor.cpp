#include "BME680Sensor.h"
#include "Logging.h"
#include "Utilities.h"
#include "PersistedConfiguration.h"

#define BME680_CHIP_ID_REG     0xD0
#define BME680_RESET_REG       0xE0
#define BME680_STATUS_REG      0x1D

#define BME680_idac_heat_0_REG      0x50
#define BME680_res_heat_0_REG       0x5A
#define BME680_gas_wait_0_REG       0x64

#define BME680_ctrl_gas_1_REG       0x71
#define BME680_ctrl_hum_REG         0x72
#define BME680_ctrl_meas_REG        0x74
#define BME680_config_REG           0x75

#define BME680_par_g1_REG           0xED
#define BME680_par_g2_LSB_REG       0xEB
#define BME680_par_g2_MSB_REG       0xEC
#define BME680_par_g3_REG           0xEE
#define BME680_res_heat_range_REG   0x02
#define BME680_res_heat_range_MASK  0b00110000
#define BME680_res_heat_range_SHIFT 4
#define BME680_res_heat_val_REG     0x00

#define BME680_par_t1_LSB_REG       0xE9
#define BME680_par_t1_MSB_REG       0xEA
#define BME680_par_t2_LSB_REG       0x8A
#define BME680_par_t2_MSB_REG       0x8B
#define BME680_par_t3_REG           0x8C

#define BM680_par_p1_LSB_REG        0x8E
#define BM680_par_p1_MSB_REG        0x8F

#define BME680_range_switching_error_REG        0x04

#define BME680_HUMIDITY_OS_MASK         0x07
#define BME680_HUMIDITY_OS_SHIFT        0
#define BME680_TEMPERATURE_OS_MASK      0xE0
#define BME680_TEMPERATURE_OS_SHIFT     5
#define BME680_PRESSURE_OS_MASK         0x1C
#define BME680_PRESSURE_OS_SHIFT        2
#define BME680_IIR_FILTER_COEF_MASK     0x1C
#define BME680_IIR_FILTER_COEF_SHIFT    2

#define BME680_nb_conv_MASK             0x0F
#define BME680_run_gas_MASK             0x10
#define BME680_run_gas_ON               0x10
#define BME680_run_gas_OFF              0x00


BME680Sensor::BME680Sensor(PersistedConfiguration& config)
    :   SensorBase(config),
        _sensorConfigured(false),
        _measurementCompletionMillis(0)
{
    if (!begin())
    {
        PRINTLN_ERROR(F("ERROR unable to init BME680"));
        setIsFound(false);
    } else {
        setIsFound(true);
    }
}

BME680Sensor::~BME680Sensor()
{

}

bool BME680Sensor::begin(void)
{
    uint8_t chip_id = 0;
    if (!readRegister(BME680_CHIP_ID_REG, chip_id)) {

        return false;
    }
    if (chip_id != 0x61) {
        PRINT_ERROR(F("ERROR BME680 wrong chip ID"));
        return false;
    }
    PRINTLN_INFO(F("Found BME680"));
    return true;
}

void BME680Sensor::reset(void)
{
    if (!writeRegister(BME680_RESET_REG, 0xB6)) {
        PRINTLN_ERROR(F("ERROR can't reset BME680"));
        return;
    }
    delay(10);
    PRINTLN_DEBUG(F("BME680 reset"));
}

void BME680Sensor::setup(void)
{
    PRINTLN_DEBUG(F("Setup BME680"));

    setSensorConfig();

    _sensorConfigured = true;
}

void BME680Sensor::setSensorConfig(void){
    // Select oversampling for T, P and H
    uint8_t tempOS = ((uint8_t)getTemperatureOversampling()) << BME680_TEMPERATURE_OS_SHIFT;
    uint8_t humidOS = ((uint8_t)getHumidityOversampling()) << BME680_HUMIDITY_OS_SHIFT;
    uint8_t pressureOS = ((uint8_t)getPressureOversampling()) << BME680_PRESSURE_OS_SHIFT;

    if (!updateRegister(BME680_ctrl_hum_REG, BME680_HUMIDITY_OS_MASK, humidOS)) {
        PRINTLN_DEBUG(F("ERROR set humid OS"));
        return;
    }
    if (!updateRegister(BME680_ctrl_meas_REG, BME680_TEMPERATURE_OS_MASK|BME680_PRESSURE_OS_MASK, tempOS|pressureOS )) {
        PRINTLN_DEBUG(F("ERROR set temp or press OS"));
        return;
    }

    // Select IIR filter for temperature sensor
    uint8_t iirCoef = (uint8_t)getIIRFilterCoef() << BME680_IIR_FILTER_COEF_SHIFT;
    if (!updateRegister(BME680_config_REG, BME680_IIR_FILTER_COEF_MASK, iirCoef)) {
        PRINTLN_DEBUG(F("ERROR set IIR"));
        return;
    }

    // Define heater-on time
    if (!writeRegister(BME680_gas_wait_0_REG, calculateHeaterDuration(getGasHeatDuration()))) {
        PRINTLN_DEBUG(F("ERROR set heat time"));
        return;
    }

    // Enable gas coversion
    updateTemperatureTargetResistance(getGasHeaterTemperature(), 25);

    // Select index of heater set-point and turn on gas
    updateRegister(BME680_ctrl_gas_1_REG, BME680_nb_conv_MASK | BME680_run_gas_MASK, 0 | BME680_run_gas_ON ); // index 0 & on gas heater
}

uint8_t BME680Sensor::calculateHeaterDuration(uint16_t dur)
{
	uint8_t factor = 0;
	uint8_t durval;

	if (dur >= 0xfc0) {
		durval = 0xff; /* Max duration*/
	} else {
		while (dur > 0x3F) {
			dur = dur / 4;
			factor += 1;
		}
		durval = (uint8_t) (dur + (factor * 64));
	}

	return durval;
}

bool BME680Sensor::updateTemperatureTargetResistance(int16_t target_temp, int16_t amb_temp)
{
    // TODO read ambient temperature from sensor
    uint8_t res_heat_x;
    if (!calculateTemperatureTargetResistance(target_temp, amb_temp, res_heat_x)) {  // target 300 degrees
        PRINTLN_DEBUG(F("ERROR calc res_reat_x"));
        return false;
    }

    PRINT_DEBUG(F("    res_heat_x = 0x"));
    PRINT_HEX_DEBUG(res_heat_x);
    PRINT_DEBUG(F("\n"));
    writeRegister(BME680_res_heat_0_REG, res_heat_x);

    return true;
}
bool BME680Sensor::calculateTemperatureTargetResistance(int16_t target_temp, int16_t amb_temp, uint8_t& out_res_heat_x )
{
    // first get calibration values from registers
    int32_t par_g1, par_g2, par_g3, res_heat_range;
    int8_t res_heat_val;
    uint8_t reg_value;

    if (!readRegister(BME680_par_g1_REG, reg_value)) {
        return false;
    }
    par_g1 = reg_value;

    if (!readRegister(BME680_par_g2_LSB_REG, reg_value)) {
        return false;
    }
    par_g2 = reg_value;
    readRegister(BME680_par_g2_MSB_REG, reg_value);
    par_g2 += (uint16_t)reg_value*256;

    readRegister(BME680_par_g3_REG, reg_value);
    par_g3 = reg_value;

    readRegister(BME680_res_heat_range_REG, reg_value);
    res_heat_range = (reg_value&BME680_res_heat_range_MASK) >> BME680_res_heat_range_SHIFT;

    readRegister(BME680_res_heat_val_REG, reg_value);
    res_heat_val = (int8_t)reg_value;

    // calculate value using interger math
    int32_t var1 = (((int32_t)amb_temp * par_g3) / 10) << 8;
    int32_t var2 = (par_g1 + 784) * (((((par_g2 + 154009) * (int32_t)target_temp * 5) / 100) + 3276800) / 10);
    int32_t var3 = var1 + (var2 >> 1);
    int32_t var4 = (var3 / (res_heat_range + 4));
    int32_t var5 = (131 * res_heat_val) + 65536;
    int32_t res_heat_x100 = (int32_t)(((var4 / var5) - 250) * 34);

    out_res_heat_x = (uint8_t)((res_heat_x100 + 50) / 100);

    return true;
}

bool BME680Sensor::isActive(void) const
{
    // return false;
   return SensorBase::isActive() && _sensorConfigured;
}

void BME680Sensor::startMeasurementProcess(void)
{
    if (isActive()) {
        // Trigger forced mode measurement
        if (!updateRegister(BME680_ctrl_meas_REG, 0b00000011, 0x01)) { // forced mode
            PRINTLN_ERROR(F("ERROR start BME660"));
            return;
        }
    }
}

uint16_t BME680Sensor::calculateMeasurmentDuration(void) const
{
    // TODO calculate more rpecise measurement delay.
    return 300;
}

const uint8_t* BME680Sensor::getCurrentMeasurementBuffer(void)
{
    // normally the BME680 measurement process can be done asynchronously, allowing the MCU to do other
    // things. But the AmbaSat-1 will brown out if both the BME680 heater and the RFM95 transmitter are
    // active at the same time. This likely could be fixed with a capacitor or two. Or we can fix it in software
    // by running the BME680 synchronously with everything else the AmbaSat-1 is doing. Software fix it is.
    if (isActive()) {
        startMeasurementProcess();
    } else {
        return nullptr;
    }
    delay(calculateMeasurmentDuration());

    uint8_t reg_value;
    if (!readRegister(BME680_STATUS_REG, reg_value)) {
        PRINTLN_ERROR(F("ERROR read BME680 status"));
        return nullptr;
    }

    bool isDone = ((reg_value&0b0010000) == 0);
    if (!isDone) {
        PRINT_DEBUG(F("  waiting BME680"));
        while (!isDone) {
            PRINT_DEBUG(F("."));
            if (!readRegister(BME680_STATUS_REG, reg_value)) {
                PRINTLN_ERROR(F("ERROR reading BME680"));
                return nullptr;
            }
            isDone = ((reg_value&0b0010000) == 0);
            delay(1);
        }
        PRINTLN_DEBUG(F("\n  BME680 done"));
    }

    // get temp, pressure, and humidity ADC values. The buffer map is
    //  buffer idx  | register | description
    //  ------------+----------+----------------------
    //            0 |   0x1F   | press_adc MSB
    //            1 |   0x20   | press_adc LSB
    //            2 |   0x21   | press_adc XLSB <7:4>
    //            3 |   0x22   | temp_adc MSB
    //            4 |   0x23   | temp_adc LSB
    //            5 |   0x24   | temp_adc XLSB <7:4>
    //            6 |   0x25   | hum_adc MSB
    //            7 |   0x26   | hum_adc LSB
    //            8 |   0x2A   | gas_r<9:2>
    //            9 |   0x2B   | gas_r<1:0>, gas_valid_r, heat_stab_r, gas_range_r
    //

    uint8_t adc_buffer[10];
    memset(adc_buffer, 0x00, 8);
    readRegisters(0x1F,adc_buffer, 8);
    readRegisters(0x2A,&adc_buffer[8], 2);

#if LOG_LEVEL >= LOG_LEVEL_DEBUG
    PRINT_DEBUG(F("  ADC = "));
    print_buffer(adc_buffer, 8);
#endif

    int32_t t_fine;
    int32_t temp_comp = calibratedTemperatureReading(adc_buffer[3], adc_buffer[4], adc_buffer[5], t_fine);
    int32_t press_comp = calibratedPressureReading(adc_buffer[0], adc_buffer[1], adc_buffer[2], t_fine);
    int32_t hum_comp = calibratedHumidityReading(adc_buffer[6], adc_buffer[7], temp_comp);
    int32_t gas_res = calibratedGasResistance(adc_buffer[8], adc_buffer[9]);

    PRINT_DEBUG(F("  temp = "));
    PRINT_DEBUG( temp_comp );
    PRINT_DEBUG(F(", norm temp = "));
#if LOG_CELSIUS_TEMP == 0
    PRINT_DEBUG( (((float)temp_comp*9.0/5.0/100.0) + 32.0) );
    PRINT_DEBUG(F(" °F, pres = "));
#else
    PRINT_DEBUG((float)temp_comp/100.0);
    PRINT_DEBUG(F(" °C, pressure = "));
#endif
    PRINT_DEBUG(press_comp);
    PRINT_DEBUG(F(", hum = "));
    PRINT_DEBUG(hum_comp);
    PRINT_DEBUG(F(", gas Ω = "));
    PRINT_DEBUG(gas_res);
    PRINT_DEBUG(F("\n"));

    // now that we have a current temperature, update the gas resitance sensor calibration
    updateTemperatureTargetResistance(getGasHeaterTemperature(), temp_comp/100);

    //
    // Transmit buffer format:
    //
    //      int16_t    - temperate in celsius*100
    //      int32_t    - pressure in Pa
    //      int16_t    - humidity in %*100
    //      int32_t    - gas resistance in ohms
    //      uint8_t    - temperature oversampling (high nibble) and humidity oversapling (low nibble)
    //      uint8_t    _ pressure oversampling (high nibble) and IIR filter coef (low nibble)
    //      uint16_t   - gas heater duration used for this measurment
    //      uint16_t   - gas heater temperature used for this measurement
    //  Total buffer size = 18
    //

    memset(_buffer, 0, BME680_RESULTS_BUFFER_SIZE);
    hton_int16(temp_comp, &_buffer[0]);
    hton_int32(press_comp, &_buffer[2]);
    hton_int16(hum_comp/10, &_buffer[6]);
    hton_int32(gas_res, &_buffer[8]);

    _buffer[12] = (getTemperatureOversampling() << 4)&0xF0;
    _buffer[12] |= getHumidityOversampling()&0x0F;
    _buffer[13] = (getPressureOversampling() << 4)&0xF0;
    _buffer[13] |= getIIRFilterCoef()&0x0F;
    hton_int16(getGasHeatDuration(), &_buffer[14]);
    hton_int16(getGasHeaterTemperature(), &_buffer[16]);
    return _buffer;
}

int32_t BME680Sensor::calibratedTemperatureReading(uint8_t temp_adc_msb, uint8_t temp_adc_lsb, uint8_t temp_adc_xlsb, int32_t& t_fine)
{
    // calculate temp_adc;
    int32_t temp_adc = ((temp_adc_xlsb&0b11110000)/16) + ((int32_t)temp_adc_lsb*16) + ((int32_t)temp_adc_msb*4096);
    // first get calibration values from registers
    int32_t par_t1, par_t2, par_t3;
    uint8_t reg_value;

    if (!readRegister(BME680_par_t1_LSB_REG, reg_value)) {
        return 0xFFFFFFFF;
    }
    par_t1 = reg_value;
    if (!readRegister(BME680_par_t1_MSB_REG, reg_value)) {
        return 0xFFFFFFFF;
    }
    par_t1 += (int32_t)reg_value*256;
    if (!readRegister(BME680_par_t2_LSB_REG, reg_value)) {
        return 0xFFFFFFFF;
    }
    par_t2 = reg_value;
    if (!readRegister(BME680_par_t2_MSB_REG, reg_value)) {
        return 0xFFFFFFFF;
    }
    par_t2 += (int32_t)reg_value*256;
     if (!readRegister(BME680_par_t3_REG, reg_value)) {
        return 0xFFFFFFFF;
    }
    par_t3 = reg_value;

    // calculate
    int64_t var1 = ((int32_t)temp_adc >> 3) - ((int32_t)par_t1 << 1);
    int64_t var2 = (var1 * (int32_t)par_t2) >> 11;
    int64_t var3 = ((((var1 >> 1) * (var1 >> 1)) >> 12) * ((int32_t)par_t3 << 4)) >> 14;
    t_fine = var2 + var3;
    int32_t temp_comp = ((t_fine * 5) + 128) >> 8;

    return temp_comp;
}

int32_t BME680Sensor::calibratedPressureReading(uint8_t press_adc_msb, uint8_t press_adc_lsb, uint8_t press_adc_xlsb, int32_t t_fine)
{
    // calculate press_adc;
    int32_t press_adc = ((press_adc_xlsb&0b11110000)/16) + ((int32_t)press_adc_lsb*16) + ((int32_t)press_adc_msb*4096);

    // first get calibration values from registers
    int32_t par_p1, par_p2, par_p3, par_p4, par_p5, par_p6, par_p7, par_p8, par_p9, par_p10;
    uint8_t buffer[19];

    if (!readRegisters(BM680_par_p1_LSB_REG, buffer, 19)) {
        return 0xFFFFFFFF;
    }
    par_p1 = buffer[0] + (int32_t)buffer[1]*256;
    par_p2 = buffer[2] + (int32_t)buffer[3]*256;
    par_p3 = buffer[4];
    par_p4 = buffer[6] + (int32_t)buffer[7]*256;
    par_p6 = buffer[9];
    par_p7 = buffer[8];
    par_p8 = buffer[14] + (int32_t)buffer[15]*256;
    par_p9 = buffer[15] + (int32_t)buffer[17]*256;
    par_p9 = buffer[18];

    // calculate
    int32_t var1 = ((int32_t)t_fine >> 1) - 64000;
    int32_t var2 = ((((var1 >> 2) * (var1 >> 2)) >> 11) * (int32_t)par_p6) >> 2;
    var2 = var2 + ((var1 * (int32_t)par_p5) << 1);
    var2 = (var2 >> 2) + ((int32_t)par_p4 << 16);
    var1 = (((((var1 >> 2) * (var1 >> 2)) >> 13) * ((int32_t)par_p3 << 5)) >> 3) + (((int32_t)par_p2 * var1) >> 1);
    var1 = var1 >> 18;
    var1 = ((32768 + var1) * (int32_t)par_p1) >> 15;
    int32_t press_comp = 1048576 - press_adc;
    press_comp = (uint32_t)((press_comp - (var2 >> 12)) * ((uint32_t)3125));
    if (press_comp >= ((int32_t)1 << 30)) {
        press_comp = ((press_comp / (uint32_t)var1) << 1);
    } else {
        press_comp = ((press_comp << 1) / (uint32_t)var1);
    }
    var1 = ((int32_t)par_p9 * (int32_t)(((press_comp >> 3) * (press_comp >> 3)) >> 13)) >> 12;
    var2 = ((int32_t)(press_comp >> 2) * (int32_t)par_p8) >> 13;
    int32_t var3 = ((int32_t)(press_comp >> 8) * (int32_t)(press_comp >> 8) * (int32_t)(press_comp >> 8) * (int32_t)par_p10) >> 17; 

    press_comp = (int32_t)(press_comp) + ((var1 + var2 + var3 + ((int32_t)par_p7 << 7)) >> 4);

    return press_comp;
}

int32_t BME680Sensor::calibratedHumidityReading(uint8_t hum_adc_msb, uint8_t hum_adc_lsb, int32_t temp_comp)
{
    // calculate hum_adc;
    int32_t hum_adc = hum_adc_lsb + (int32_t)hum_adc_msb*256;


    // first get calibration values from registers
    int32_t par_h1, par_h2, par_h3, par_h4, par_h5, par_h6, par_h7;
    uint8_t buffer[8];

    if (!readRegisters(0xE1, buffer, 8)) {
        return 0xFFFFFFFF;
    }
    // the data sheet has an error in it. the LSB are bite <3:0> at register 0xE2
    par_h1 = (buffer[1]&0x0F) + (int32_t)buffer[2]*16;
    par_h2 = ((buffer[1]&0xF0) >> 4) + (int32_t)buffer[0]*16;
    par_h3 = buffer[3];
    par_h4 = buffer[4];
    par_h5 = buffer[5];
    par_h6 = buffer[6];
    par_h7 = buffer[7];

    // calculate
    int32_t temp_scaled = temp_comp;
    int32_t var1 = (int32_t)hum_adc - (int32_t)((int32_t)par_h1 << 4) - (((temp_comp * (int32_t)par_h3) / ((int32_t)100)) >> 1);
    int32_t var2 = ((int32_t)par_h2 * (((temp_scaled * (int32_t)par_h4) / ((int32_t)100)) + (((temp_scaled * ((temp_scaled * (int32_t)par_h5) / ((int32_t)100))) >> 6) / ((int32_t)100)) + ((int32_t)(1 << 14)))) >> 10;
    int32_t var3 = var1 * var2;
    int32_t var4 = (((int32_t)par_h6 << 7) + ((temp_scaled * (int32_t)par_h7) / ((int32_t)100))) >> 4;
    int32_t var5 = ((var3 >> 14) * (var3 >> 14)) >> 10;
    int32_t var6 = (var4 * var5) >> 1;
    int32_t hum_comp = (var3 + var6) >> 12;
    hum_comp = (((var3 + var6) >> 10) * ((int32_t) 1000)) >> 12;

    return hum_comp;
}

int32_t BME680Sensor::calibratedGasResistance(uint8_t gas_adc_msb, uint8_t gas_adc_lsb)
{
    //
    // The following is a compression of the memory required to represent the 16 position array of 4-byte ints paramters
    // required for the algorithm. This array has repeated values, so the compression is to have a array of the
    // distinct values, then a 16 position array of 1 byte indexes into the distinct values. A compile macro
    // is used to easily handle the index dereferenceing.
    //
    // The original parameter array:
    // static const uint32_t const_array1_int[16] = {
    //      UINT32_C(2147483647), UINT32_C(2147483647), UINT32_C(2147483647), UINT32_C(2147483647),
    //      UINT32_C(2147483647), UINT32_C(2126008810), UINT32_C(2147483647), UINT32_C(2130303777),
    //      UINT32_C(2147483647), UINT32_C(2147483647), UINT32_C(2143188679), UINT32_C(2136746228),
    //      UINT32_C(2147483647), UINT32_C(2126008810), UINT32_C(2147483647), UINT32_C(2147483647)
    // };
    //

    static const uint32_t const_array1_int_values[5] = {
        UINT32_C(2147483647),
        UINT32_C(2126008810),
        UINT32_C(2130303777),
        UINT32_C(2143188679),
        UINT32_C(2136746228),
    };

    static const uint8_t const_array1_int_indexes[16] = {
        0, 0, 0, 0,
        0, 1, 0, 2,
        0, 0, 3, 4,
        0, 1, 0, 0,
    };

    #define GET_const_array1_int(i) const_array1_int_values[const_array1_int_indexes[i]]

    // this array doesn't have repeated values so not worth compressing
    static const uint32_t const_array2_int[16] = {
        UINT32_C(4096000000), UINT32_C(2048000000), UINT32_C(1024000000), UINT32_C(512000000),
        UINT32_C(255744255), UINT32_C(127110228), UINT32_C(64000000), UINT32_C(32258064),
        UINT32_C(16016016), UINT32_C(8000000), UINT32_C(4000000), UINT32_C(2000000),
        UINT32_C(1000000), UINT32_C(500000), UINT32_C(250000), UINT32_C(125000)
    };

    int16_t gas_adc = (int16_t)(gas_adc_lsb/64) + (int16_t)gas_adc_msb*4;
    uint8_t gas_range = gas_adc_lsb&0b00001111;

    uint8_t range_switching_error;
    if (!readRegister(BME680_range_switching_error_REG, range_switching_error)) {
        return 0xFFFFFFFF;
    }

    // calculate
    int64_t var1 = (int64_t)(((1340 + (5 * (int64_t)range_switching_error)) * ((int64_t)GET_const_array1_int(gas_range))) >> 16);
    int64_t var2 = (int64_t)(gas_adc << 15) - (int64_t)((int64_t)1 << 24) + var1;

    int32_t gas_res = (int32_t)(( ((int64_t)(const_array2_int[gas_range] * (int64_t)var1) >> 9) + (var2 >> 1)) / var2);

    return gas_res;
}

//
// Sensor Configuration Delegate
//

#define OFFSET_TEMP_OVERSAMPLING            0       // 1 byte
#define OFFSET_HUMIDITY_OVERSAMPLING        1       // 1 byte
#define OFFSET_PRESSURE_OVERSAMPLING        2       // 1 byte
#define OFFSET_IIR_COEF                     3       // 1 byte
#define OFFSET_GAS_HEAT_DURATION            4       // 2 bytes
#define OFFSET_GAS_HEAT_TEMP                6       // 2 bytes

#define CONFIG_BME680SENSOR_BLOCK_SIZE      8

uint8_t BME680Sensor::configBlockSize( void ) const
{
    return CONFIG_BME680SENSOR_BLOCK_SIZE;
}

void BME680Sensor::setDefaultValues(void)
{
    setTemperatureOversampling(BME680_OVERSAMPLING_8x);
    setHumidityOversampling(BME680_OVERSAMPLING_8x);
    setPressureOversampling(BME680_OVERSAMPLING_8x);
    setIIRFilterCoef(BME_FILTER_COEF_3);
    setGasHeatTemperature(320);
    setGasHeatDuration(150);
}

void BME680Sensor::loadConfigValues(void)
{
    // the following compile check ensures that the payload buffer is large enough to be used here
    #if BME680_RESULTS_BUFFER_SIZE < CONFIG_BME680SENSOR_BLOCK_SIZE
    #error Cannot reuse BME680 results buffer to load configuration because it is too small
    #else
    // this approach reduces code size. Reusing payload buffer to conserve memory.
    eeprom_read_block(_buffer, (uint8_t *)getEEPROMBaseAddress(), CONFIG_BME680SENSOR_BLOCK_SIZE);
    _temperatureOversampling = (BME680SensorOversamplingSetting)_buffer[OFFSET_TEMP_OVERSAMPLING];
    _humidityOversampling = (BME680SensorOversamplingSetting)_buffer[OFFSET_HUMIDITY_OVERSAMPLING];
    _pressureOversampling = (BME680SensorOversamplingSetting)_buffer[OFFSET_PRESSURE_OVERSAMPLING];
    _iirCoefSetting = (BME680IIRFilterCoefSetting)_buffer[OFFSET_IIR_COEF];
    memcpy(&_gasProfile0_duration, &_buffer[OFFSET_GAS_HEAT_DURATION], sizeof(uint16_t));
    memcpy(&_gasProfile0_targetTemp, &_buffer[OFFSET_GAS_HEAT_TEMP], sizeof(uint16_t));
    #endif
}

void BME680Sensor::writeConfigToBuffer( uint8_t* bufferBaseAddress) const
{
    bufferBaseAddress[OFFSET_TEMP_OVERSAMPLING] = _temperatureOversampling;
    bufferBaseAddress[OFFSET_HUMIDITY_OVERSAMPLING] = _humidityOversampling;
    bufferBaseAddress[OFFSET_PRESSURE_OVERSAMPLING] = _pressureOversampling;
    bufferBaseAddress[OFFSET_IIR_COEF] = _iirCoefSetting;
    *(int16_t*)&bufferBaseAddress[OFFSET_GAS_HEAT_DURATION] = _gasProfile0_duration;
    *(int16_t*)&bufferBaseAddress[OFFSET_GAS_HEAT_TEMP] = _gasProfile0_targetTemp;
}

void BME680Sensor::setTemperatureOversampling(BME680SensorOversamplingSetting setting)
{
    eeprom_update_byte((uint8_t *)(getEEPROMBaseAddress()+OFFSET_TEMP_OVERSAMPLING), setting);
    _temperatureOversampling = setting;
}

void BME680Sensor::setHumidityOversampling(BME680SensorOversamplingSetting setting)
{
    eeprom_update_byte((uint8_t *)(getEEPROMBaseAddress()+OFFSET_HUMIDITY_OVERSAMPLING), setting);
    _humidityOversampling = setting;
}

void BME680Sensor::setPressureOversampling(BME680SensorOversamplingSetting setting)
{
    eeprom_update_byte((uint8_t *)(getEEPROMBaseAddress()+OFFSET_PRESSURE_OVERSAMPLING), setting);
    _pressureOversampling = setting;
}

void BME680Sensor::setIIRFilterCoef(BME680IIRFilterCoefSetting setting)
{
    eeprom_update_byte((uint8_t *)(getEEPROMBaseAddress()+OFFSET_IIR_COEF), setting);
    _iirCoefSetting = setting;
}

void BME680Sensor::setGasHeatDuration(int16_t setting, uint8_t profile)
{
    // for now, profile is ignored
    eeprom_update_word((uint16_t *)(getEEPROMBaseAddress()+OFFSET_GAS_HEAT_DURATION), setting);
    _gasProfile0_duration = setting;
}

void BME680Sensor::setGasHeatTemperature(int16_t setting, uint8_t profile)
{
    // for now, profile is ignored
    eeprom_update_word((uint16_t *)(getEEPROMBaseAddress()+OFFSET_GAS_HEAT_TEMP), setting);
    _gasProfile0_targetTemp = setting;
}

#ifdef ENABLE_AMBASAT_COMMANDS
uint8_t BME680Sensor::handleCommand(uint16_t cmdSequenceID, uint8_t command, uint8_t* recievedData, uint8_t recievedDataLen)
{
    if ((command >= 0x01)&&(command <= 0x03)) {
        if (recievedDataLen != 1) {
            return CMD_STATUS_BAD_DATA_LEN;
        }
        uint8_t value = (*recievedData)&0b00000111;
        if (value > BME680_OVERSAMPLING_16x) {
            value = BME680_OVERSAMPLING_16x;
        }
        BME680SensorOversamplingSetting oversampling = (BME680SensorOversamplingSetting)value;
        if (command == 0x01) {
            // Set Temperature Oversampling
            PRINT_DEBUG(F("  Temp set = "));
            PRINT_DEBUG(oversampling);
            PRINT_DEBUG(F("\n"));
            setTemperatureOversampling(oversampling);
        } else if (command == 0x02) {
            // Set Pressure Oversampling
            PRINT_DEBUG(F("  Pres set = "));
            PRINT_DEBUG(oversampling);
            PRINT_DEBUG(F("\n"));
            setPressureOversampling(oversampling);
        } else {
            // Set Hummidity Oversampling
            PRINT_DEBUG(F("  Hum set = "));
            PRINT_DEBUG(oversampling);
            PRINT_DEBUG(F("\n"));
            setHumidityOversampling(oversampling);
        }
    } else if (command == 0x04) {
        // Set IIR Filter Coefficient
        if (recievedDataLen != 1) {
            return CMD_STATUS_BAD_DATA_LEN;
        }
        BME680IIRFilterCoefSetting coeff = (BME680IIRFilterCoefSetting)((*recievedData)&0b00000111);
        PRINT_DEBUG(F("  IIR set = "));
        PRINT_DEBUG(coeff);
        PRINT_DEBUG(F("\n"));
        setIIRFilterCoef(coeff);
    } else if (command == 0x05) {
        // Set Gas Heater Heat Time
        if (recievedDataLen != 2) {
            return CMD_STATUS_BAD_DATA_LEN;
        }
        int16_t millis = ntoh_int16(recievedData);
        if ((millis < 1)  || (millis > 4032)) {
            return CMD_STATUS_BAD_PARAM;
        }
        PRINT_DEBUG(F("  Heat time = "));
        PRINT_DEBUG(millis);
        PRINT_DEBUG(F(" ms\n"));
        setGasHeatDuration(millis);
    } else if (command == 0x06) {
        // Set Gas Heater Heat Temperature
        // TODO: convert this to a 1 byte parameter that indicates offset from 200
        if (recievedDataLen != 2) {
            return CMD_STATUS_BAD_DATA_LEN;
        }
        int16_t plate_temp = ntoh_int16(recievedData);
        if ((plate_temp < 200)  || (plate_temp > 400)) {
            return CMD_STATUS_BAD_PARAM;
        }
        PRINT_DEBUG(F("  Heat temp = "));
        PRINT_DEBUG(plate_temp);
        PRINT_DEBUG(F("\n"));
        setGasHeatTemperature(plate_temp);
    } else if (command == 0x07) {
        PRINT_DEBUG(F("  default\n"));
        setDefaultValues();
    }
    else {
        return CMD_STATUS_UNKNOWN_CMD;
    }
    this->_config.updateCRC();
    setSensorConfig();
    return CMD_STATUS_SUCCESS;
}
#endif