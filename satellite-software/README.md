# AmbaSat-1 Flight Software
This repository contains the flight software for the AmbaSat-1 developped my Michael Kamprath. This software is free and open source, and anybody is welcome to use it on their AmbaSat-1. The goal of this flight software are:

* Maximize Telemetry Received - We are all building these AmbaSat-1 devices because it is cool, and the climax of our efforts will be when we receive from space telemetry from the satellite we built. I want to maximize that. Of course, there are constraints to be considered, like fair use of The Things Network (TTN), but within those constraints I want to get as much data as I can possibly can.
* Robustness - Space is a harsh operating environment. Furthermore, launches can jostle satellites and break components. I want my satellite to do the best it can despite damage incurred during ascent or in orbit. This means being tolerant of sensor failures and even doing on board testing.  Of course, software can't compensate for catastrophic hardware failures, but the software should strive to maximize data received by working around a malfunctioning sensor (if it has one).
* Keep the Code clean and Understandable - Software that is built from example code that itself was copy and pasted from other examples has a way of getting messy with vestigial code that serves no purpose. The goal here is to build the software from the ground up, purpose built for the AmbaSat-1.

## How to Use
This software is ready to go for the AmbaSat basic sensors, Voltage and LSM9DS1, with minimal set up. To build this software for your AmbaSat-1, first do the following:

1. Set up an application in your The Things Network account. Create a sensor, which will be the AmbaSat-1. Set this sensor up for ABP. Note that a decoder to use in your The Things Network application configuation is [provided elsewhere in this repository](../ground-software/ttn-payload-decoders/payload-decoders.js).
2. Set the Network Session Key, LoRaWAN AppSKey, the LoRaWAN end-device address, and the `AMBASAT_MISSION_SENSOR` values in the `AmbaSat1Config.h` header file. Find instructions for obtaining these values for ABP activation at The Things Network site. Do not check this edited file into source control, as these values should be treated as secrets.
3. Change the radio frequency configuration in the `platformio.ini` file. If you are using the USA frequency plan of 915 MHz, the frequency configuration should be `CFG_us915`. If you are using the radio provided in the AmbaSat-1 kits, which use the European frequency plan of 868 MHz, the frequency configuration should be `CFG_eu868`. Note that the version of your satellite that will launch will likely have the European frequency plan radio and the flight software should be configured for that when finalizing your satellite.
4. Set the `monitor_port` and `upload_port` options in the `platformio.ini` file to the USB device of the UART being used to connect to your AmbaSat-1 satellite.


# Design
Below are the intended requirements for the finalized AmbaSat-1 flight software. This is very much a `Work in Progress`.

## Requirements

### Hardware Configuration
The AmbaSat-1 picosat is cuilt with an external 4 MHz resonator to drive the ATmega328P microcontroller, however the default fuse settings for the ATmega328P has it using its own internal clock at 1 MHz. This is done to minimize the power requirements of the AmbaSat-1 picosat. However, utilizing the internal clock precludes the AmbaSata-1 from recieving downlinks from The Things Network due to the internal clock not being accurate enough to properly detect the downlink radio transmission. As a result, two hardware configurations will be enabled in this code base, and are identified by their board description files ([found here](https://github.com/michaelkamprath/ambasat1-flight-software/tree/master/boards/)):

* **AmbaSat-1** - This is the original board definition provided by the AmbaSat-1 makers. This definition expects the ATmega328P to be configured to use the internal clock at 1 Mhz. The ATmega328P fuses for this board are `(E:FE, H:D6, L:62)`.
* **AmbaSat-1b** -  This board definition enables the use of the external 4 MHz resonator on the AmbaSat-1 picosat and a BOD of 1.8V. This hardware configuration is required if you wish to enable commands to the AmbaSat-1 via downlinks. The ATmega328P fuses for this board are `(E:FE, H:D6, L:F7)`.

The sole difference between these hardware configurations is the fuse configuration the ATmega328P is burned with, and the corresponding configuration to the MiniCore bootloader that is burned on the the chip.

### Telemetry and Status Uplinks

#### Uplink Ports
The LoRaWAN protocol affords different communication ports to allow for transmitting different types of payloads. Each port should be used for a specific type of data packet to be uplinked. Each port should have a configurable downlink rate, and the satellite should have a maximum overall downlink rate (the sum of all port downlinks). The specific port mappings will be:

| Port | Description | Comments |
|:-:|:--|:--|
| 1 | Satellite Status | Information about the status of the AmbaSat-1 (e.g., reboot count, voltage, etc) |
| 2 | LSM9DS1 | Acceleration, Rotation, and Magnetic Field |
| 3 | Sensor 1 - SHT30-DIS-F2.5KS | Humidity and Temperature |
| 4 | Sensor 2 - STS21 | Temperature |
| 5 | Sensor 3 - BME680 | Gas, Pressure, Temp & Humidity |
| 6 | Sensor 4 - OPT3001DNPT | Ambient Light Sensor |
| 7 | Sensor 5 - ZMOD4410AI1V | TVOC (Total Volatile Organic Compounds) Sensor |
| 8 | Sensor 6 - Si1132 | UV Light Sensor |
| 9 | Sensor 7 - CCS811B-JOPD500 | TVOC & CO2 Sensor |
| 10 | Sensor 8 - TESEO-LIV3R | Global Navigation Satellite System (GNSS) |
| 11 | CMD Status | Report the status of a command request |


Since AmbaSat only supports one sensor other than the LSM9DS1, clearly not all of the ports will be in use when in flight. However, so that this software can be generally used, support for all sensor types will be created. In order to minimize the amount of space is required on the ATMEGA328, all sensor support will be configured with a compiler macro to either activate or deactivate the sensor specific code.

##### Satellite Status  Uplink Format
The statellite status uplink will report the following attributes of the AmbaSat-1 picosat:

1. **Reboot Count** - In order to get a sense of the frequency of reboots the AmbaSat-1 experiences due to the solar panels not facing sunlight, either due to orbiting to the night side of the earth or simply tumbling in space, a persisted `uint32_t` counter will be used to track the number of times the AmbaSat-1 powered up.
2. **Voltage Level** - The voltage level that the ATMEGA328 is experiencing will be reported. This will be reported as millivolts using a `uint16_t`.
3. **Sensor Status** - One `uint8_t` will be used to report the status of both the LSM9DS1 sensor and the mission sensor. The format of this by is documented below.

The Sensor Status byte will have it's individual bits set as follows:

| Bit | Description | Setting |
|:-:|:--|:--|
| 7 | Reserved | 0 |
| 6 | Reserved | 0 |
| 5 | Mission Sensor Active | 1 if active, 0 if deactivated due to commanded configuration or not found |
| 4 | Mission Sensor Found | 1 if found on I2C bus during boot, 0 if not |
| 3 | Reserved | 0 |
| 2 | Reserved | 0 |
| 1 | LSM9DS1 Active | 1 if active, 0 if deactivated due to commanded configuration or not found |
| 0 | LSM9DS1 Found | 1 if found on I2C bus during boot, 0 if not |

##### Sensor Uplink Format
The uplink data format for sensors (ports 2-10) will be specific to that sensor and documented in the sensor's code. In general, all sensors will do minimal processing of the sensor measurements in the flight software and instead defer to ground software to convert the sensor measurements to usable units. In order to make that possible, the uplink data formats should include the current sensitivity setting of the sensor, if applicable, so that the ground software knows how to interpret the measurement values.

##### CMD Status Uplink Format
The `CMD Status` Uplink will have the following format: **TBD**


#### Uplink Decoder
TBD

### Command Request Downlinks
In normal operations, the AmbaSat is not intended to be regularly sent commands to change how it operates. However, the ability to alter operational parameters of the satellite will be enabled using the TTN downlink abilities. To enable command downlinks, the AmbaSat must use the hardware configuration that enables downlinks to be received (see Hardware Configuration above).

The command will consist of at least 1 byte. This byte will be split into the high and low 4 bits. The high 4 bits will represent the actual command, and the low fit bits will represent the sensor ports the command pertains to, if relevant. As a result, there can only be 16 commands that the satellite will recognize, and there are only 15 sensor ports the satellite can support. A port value of 0 for a command indicates the command does not apply to a particular sensor. The command can have an optional second and third byte that will contain configuration data, depending on the command. 

Downlinks will use the LoRaWAN channel to indicate how the command should be routed according the this mapping:

| Channel | Command Type | Description |
|:--|:--|:--|
| 2 | Satellite | Commands that pertain to the basic operation of the AmbaSat-1 |
| 3 | LSM9DS1 | Commands that pertain to the onboard LSM9DS1 gyro sensor |
| 4 | Mission Sensor | Commands that pertain to the installed mission sensor |

The general format of the command downlink is:

| Struct Member | Name | Description |
|:--|:--|:--|
| `uint16_t` | Command Sequence ID | This is a user defined ID for the command being sent. It will be used to identify this command in any response the satellite provides. |
| `uint8_t` | Command ID | A number that indicates what command is being sent. The interpretation of this command ID is contextual to the command channel being used. |
| `uint8_t*` | Command Data | An arbitrary length of 0 or more bytes containing data specific to the command being sent. The size of format of this data blob is defined by the command sent. |

#### Satellite Commands

##### Blink LED
Because the aliens in space like blinking lights too. This command causes the LED installed on the AmbaSat-1 to blink.

* **Command ID**  : `0x01` 
* **Command Data** : A single byte is used for the blink command data. 
  * The most significant 2 bits are used to indicate the blink duration as follows:
     *  `00` = 0.1 second blinks
     *  `01` = 0.5 second blinks
     *  `10` = 1 second blinks
     *  `11` = 2 second blinks
  *  The least significant 6 bits are used to indicate the number of blinks to make. 
 
For example, the hex value of 0x84 would cause 4 blinks each 1 second long.

##### Set Sensor Telemetry Uplink Pattern
Changes the uplink pattern that the AmbaSat uses. An uplink pattern indicates what the AmbaSat will transmit each time it starts a transmission sequence.

* **Command ID**  : `0x02` 
* **Command Data** : A single byte is used to indicate which pattern to use:
  * `0x00` - All data payloads will be transmitted sequentially with this pattern: Satellite, LSM9DS1, Mission Sensor
  * `0x01` - All data payloads will be transmitted sequentially, but each uplink sequence will start with a different data payload. This pattern attempts to mitigate the satellite getting short periods of power, not long enough to transmit all payloads. 
  * `0x02` - Only a single data payload will be transmitted during each uplink sequence, rotating through the payloads type with each uplink.
  * `0x03` - Satellite + 1 data payloads will be transmitted during each uplink sequence. The second payload in addtion to the satellite payload will alternate between the LSM9DS1 and the mission sensor payloads.

##### Set Sensor Telemetry Uplink Rate
Changes the amount of time in between the telemetry uplink transmissions. During the time in between uplinks tranmissions, the AmbaSat-1 will go into low power mode. 

* **Command ID**  : `0x03` 
* **Command Data** : A single byte interpreted as a `uint8_t` value which represents the number of 8 second periods that the satellite should sleep in between uplink transmissions. 

##### Set Uplink Frame Count
The uplink frame count is used by The Things Network to validate the aunthenticity of an uplink and in turn process it. In this frame count should get out of sync, typically due to failed transmissions, resetting the uplink frame count might help. Note that this command is only useful if you have configured The Things Network to enforce frame counters. 

**IMPORTANT**: This is a risky command. Setting an incorrect value could cause The Things Network to ignore uplinks from the AmbaSat. Use with care.

* **Command ID**  : `0x04` 
* **Command Data** : Two bytes interpreted as a `int16_t` in big endian order. This is the uplink frame count that should be set. 

#### LSM9DS1 Commands

#### Mission Sensor Commands

##### Sensor 3 - BME680
The following variables will be able to be configured for the BME680 sensor:

| Command Title | Command ID |Command Data | Default Value |
|:--|:-:|:--|:-:|
| Set Temperature Oversampling| `0x01` | One byte containing one of the following enumerations:<br><br>`0b000` - No oversampling<br>`0b001` - 1x oversampling<br>`0b010` - 2x oversampling<br>`0b011` - 4x oversampling<br>`0b100` - 8x oversampling<br>`0b101` - 16x oversampling | `0b100` |
| Set Pressure Oversampling | `0x02` | One byte containing one of the following enumerations:<br><br>`0b000` - No oversampling<br>`0b001` - 1x oversampling<br>`0b010` - 2x oversampling<br>`0b011` - 4x oversampling<br>`0b100` - 8x oversampling<br>`0b101` - 16x oversampling | `0b100` |
| Set Hummidity Oversampling | `0x03` | One byte containing one of the following enumerations:<br><br>`0b000` - No oversampling<br>`0b001` - 1x oversampling<br>`0b010` - 2x oversampling<br>`0b011` - 4x oversampling<br>`0b100` - 8x oversampling<br>`0b101` - 16x oversampling | `0b100` |
| Set IIR Filter Coefficient | `0x04` | One byte containing one of the following enumerations:<br><br>`0b000` - coefficient value is 0<br>`0b001` - coefficient value is 1<br>`0b010` - coefficient value is 3<br>`0b011` - coefficient value is 7<br>`0b100` - coefficient value is 15<br>`0b101` - coefficient value is 31<br>`0b101` - coefficient value is 63<br>`0b101` - coefficient value is 127. | `0b010` |
| Set Gas Heater Heat Time | `0x05` | A two byte `int16` value indicating the amount of time in milliseconds that should be used to heat the gas sensor hot plate. The valid range is between 1 and 4032 milliseconds. | 150 |
| Set Gas Heater Heat Temperature | `0x06` | A two byte `int16` value indicating the temperature in Celsius that the gas sensor hot plate should be warmed to before making a measurement. The valid range is between 200 °C and 400 °C.  | 320 |


##### Sensor 6 - Si1132
The following variables will be able to be configured for the Si1132 sensor:

* **Visible ADC Gain** - This sets the ADC integration time for visible light measurements
* **Infrared ADC Gain** - This sets the ADC integration time for infrared light measurements
* **Visible High Signal Range** - A boolean value indicating whether the visible light in a "high signal" environment. This should be set to `true` when operating in direct sunlight.
* **Infrared High Signal Range** - A boolean value indicating whether the infrared light in a "high signal" environment. This should be set to `true` when operating in direct sunlight.

## Design Issues

### ABP
The flight software will use ABP in order to minimize the radio chatter by not requiring two way communication that OTAA requires. 

#### Frame Count Saving
As solar panels will be its only source of power, it is expected that the satellite will reboot often as it tumbles through space and goes in and out of light. The challenge here is that for ABP, TTN enforce frame counter checks as a security measure, and whenever the satellite reboots, its frame counter is otherwise reset to 0 in LMIC. One way to address this is to relax the frame counter requirements in the TTN settings for the device. TTN requests that this be done for development reasons only since it does pose a security risk. A more robust want to handle this is to save the frame counter value to the onboard flash memory.

### Flash Memory Usage
In an effort to keep the code footprint small, the following actions are taken:
* **Reducing the LMIC code footprint** - Since we are be using ABP activation, we don't need the LMIC code pertaining to OTAA. To effect this, the following compiler macros will be defined:
  * `DISABLE_JOIN`
  * `DISABLE_PING`
  * `DISABLE_BEACONS`
  * `DISABLE_MCMD_PING_SET`
  * `DISABLE_MCMD_BCNI_ANS`
* **Wrap `Serial.print()` in a macro** - In order to remove serial prints that are only really intended for debugging, a macro should be used that will allow the print statements to be compiled out in the flight build.
* **Unused sensor code not compiled** - Though this software is intended to be general purpose across all sensor types, only the configured sensor should compiled and linked when building. This will be accomplish through compiler macros.


### Radiation Hardness
Given the space environment, I would expect bits to get flipped due to the radiation. I supposed there isn't much we can do about this given the low cost nature of the AmbaSat. However, we can implement some simple data integrity checks. `TBD`

## References
The following articles and documents are useful when designing this software
* [TTN LoraWan Atmega32U4 based node – ABP version](https://primalcortex.wordpress.com/2017/10/31/ttnlorawan32u4node/)
* [Frame counters in ABP mode](https://forum.chirpstack.io/t/frame-counters-in-abp-mode/811)
* [Where in LMiC is the “fCnt” parameter stored?](https://www.thethingsnetwork.org/forum/t/where-in-lmic-is-the-fcnt-parameter-stored/3082)
* [LMIC Frame Counter Problem](https://www.robmiles.com/journal/2020/1/17/lmic-frame-counter-problem)
* [Unit Tests on Platform IO](https://www.thingforward.io/techblog/2017-07-25-starting-embedded-testing-with-platformio.html)
* [Example of How to Write to Flash Memory](https://github.com/MCUdude/MiniCore/blob/master/avr/libraries/Optiboot_flasher/examples/SerialReadWrite/SerialReadWrite.ino)