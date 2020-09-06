# TTN Payload Decoder
The `payload-decoders.js` contains the The Things Network payload decoder that should be used in conjunction with the [AmbaSat-1 flight software](../../satellite-software/). 

The decoder produces values the following units:

| Sensor | Port | Measurement | Unit |
|:--|:-:|:--|:--|
| Voltage | 1 | CPU voltage | milli-volts |
| LSM9DS1 | 2 | Acceleration | Gs (1 = standard acceleration due to gravity) |
| LSM9DS1 | 2 | Rotational Speed | Degrees per second |
| LSM9DS1 | 2 | Magnetic Field | Gauss |