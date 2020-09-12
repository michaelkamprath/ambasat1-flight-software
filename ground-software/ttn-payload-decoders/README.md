# TTN Payload Decoder
The `payload-decoders.js` contains the The Things Network payload decoder that should be used in conjunction with the [AmbaSat-1 flight software](../../satellite-software/). 

The decoder produces values the following units:

| Sensor | Port | Measurement | Unit | Description |
|:--|:-:|:--|:--|:--|
| Voltage | 1 | milli_volts | milli-volts | Voltage the ATMEGA328P is experiencing |
| LSM9DS1 | 2 | acceleration_x | G  | Acceleration in X-axis |
| LSM9DS1 | 2 | acceleration_y | G | Acceleration in Y-axis |
| LSM9DS1 | 2 | acceleration_z | G | Acceleration in Z-axis |
| LSM9DS1 | 2 | gyro_x | Degrees per second | Rotational speed about X-axis |
| LSM9DS1 | 2 | gyro_y | Degrees per second | Rotational speed about Y-axis |
| LSM9DS1 | 2 | gyro_z | Degrees per second | Rotational speed about Z-axis |
| LSM9DS1 | 2 | magnetic_x | Gauss | Magnetic field in X-axis direction |
| LSM9DS1 | 2 | magnetic_y | Gauss | Magnetic field in Y-axis direction |
| LSM9DS1 | 2 | magnetic_z | Gauss | Magnetic field in Z-axis direction |
| Si1132 | 8 | uv | Sun UV Index | ultraviolet light intensity |
| Si1132 | 8 | visible | lux | visible light intensity |
| Si1132 | 8 | ir | lux | infrared light intensity |

Notes:
* 1 G is the standard acceleration due to gravity on earth