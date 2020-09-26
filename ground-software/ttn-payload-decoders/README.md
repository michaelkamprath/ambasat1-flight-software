# TTN Payload Decoder
The `payload-decoders.js` contains the The Things Network payload decoder that should be used in conjunction with the [AmbaSat-1 flight software](../../satellite-software/). 

The decoder produces values the following units:

| Sensor | Port | Measurement | Unit | Description |
|:--|:-:|:--|:--|:--|
| Satellite Status | 1 | milli_volts | milli-volts | Voltage the ATMEGA328P is experiencing |
| Satellite Status | 1 | boot_count | integer | The number of times the AmbaSat has powered up |
| Satellite Status | 1 | LSM9DS1_found | boolean | Whether the LSM9DS1 was found on the I2C bus |
| Satellite Status | 1 | LSM9DS1_active | boolean | Whether the LSM9DS1 is sending telemetry |
| Satellite Status | 1 | mission_sensor_found | boolean | Whether the mission sensor was found on the I2C bus |
| Satellite Status | 1 | mission_sensor_active | boolean | Whether the mission sensor is sending telemetry |
| LSM9DS1 | 2 | acceleration_x | G | Acceleration in X-axis |
| LSM9DS1 | 2 | acceleration_y | G | Acceleration in Y-axis |
| LSM9DS1 | 2 | acceleration_z | G | Acceleration in Z-axis |
| LSM9DS1 | 2 | gyro_x | Degrees per second | Rotational speed about X-axis |
| LSM9DS1 | 2 | gyro_y | Degrees per second | Rotational speed about Y-axis |
| LSM9DS1 | 2 | gyro_z | Degrees per second | Rotational speed about Z-axis |
| LSM9DS1 | 2 | magnetic_x | Gauss | Magnetic field in X-axis direction |
| LSM9DS1 | 2 | magnetic_y | Gauss | Magnetic field in Y-axis direction |
| LSM9DS1 | 2 | magnetic_z | Gauss | Magnetic field in Z-axis direction |
| SHT30 | 3 | temperature | Celsius | Ambient temperature |
| SHT30 | 3 | humidity | percent | Ambient humidity |
| SHT30 | 3 | sensor_brownout_reboot | boolean | Whether SHT30 lost power and the AmbaSat did not |
| SHT30 | 3 | heater_status | boolean | Whether sensor heater is on |
| SHT30 | 3 | humidity_alert | boolean | Whether humidity value has crossed programmer thresholds |
| SHT30 | 3 | temperature_alert | boolean | Whether temperature value has crossed programmer thresholds |
| STS21 | 4 | temperature | Celsius | Ambient temperature |
| STS21 | 4 | end_of_battery | boolean | |
| STS21 | 4 | heater_status | boolean | Whether sensor heater is on |
| STS21 | 4 | measurement_resolution | integer | Number of bits used to measure temperature |
| Si1132 | 8 | uv | Sun UV Index | ultraviolet light intensity |
| Si1132 | 8 | visible | lux | visible light intensity |
| Si1132 | 8 | ir | lux | infrared light intensity |

Notes:
* 1 G is the standard acceleration due to gravity on earth