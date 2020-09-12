# AmbaSat-1 Flight Software
This repository contains the flight software for the AmbaSat-1 developped my Michael Kamprath. This software is free and open source, and anybody is welcome to use it on their AmbaSat-1. The goal of this flight software are:

* Maximize Telemetry Recieved - We are all building these AmbaSat-1 devices because it is cool, and the climax of our efforts will be when we receive from space telemetry from the satellite we built. I want to maximize that. Of course, there are contraints to be considered, like fair use of The Things Network (TTN), but within those constraints I want to get as much data as I can possibly can.
* Robustness - Space is a harsh operating environement. Furthermore, launches can jostle satellites and break components. I want my satellite to do the best it can despite damage incured during ascent or in orbit. This means being tolerant of sensor failures and even doing on board testing.  Of course, software can't compensate for catrosophic hardware failures, but the software should strive to maximize data recieved by working around a malfunctioning sensor (if it has one).
* Keep the Code clean and Understandable - Software that is built from example code that itself was copy and pasted from other examples has a way of getting messy with vestigial code that serves no purpose. The goal here is to build the software from the up purpose built for the AmbaSat-1.

## How to Use
This software is ready to go for the AmbaSat basic sensors, Voltage and LSM9DS1, with minimal set up. To build this software for your AmbaSat-1, first do the following:

1. Set up an application in your The Things Network account. Create a sensor, which will be the AmbaSat-1. Sent this sensor up for ABP. Note that a decoder to use in your The Things Network application configuation is [provided elsewhere in this repository](../ground-software/ttn-payload-decoders/payload-decoders.js).
2. Set the Network Session Key, LoRaWAN AppSKey, the LoRaWAN end-device address, and the `AMBASAT_MISSION_SENSOR` values in the `AmbaSat1Config.h` header file. Find instructions for obtaining these values for ABP activation at The Things Network site. Do not check this editted file into source control, as these values should be treated as secrets.
3. Change the radio frequency configuration in the `platformio.ini` file. If you are using the USA frequency plan of 915 MHz, the frequency configuration should be `CFG_us915`. If you are using the radio provided in the AmbaSat-1 kits, which use the European frequency plan of 868 MHz, the frequency configuration should be `CFG_eu868`. Note that the version of your satellite that will launch will likely have the European frequency plan radio and the flight software should be configured for that when finalizing your satellite.
4. Set the `monitor_port` and `upload_port` options in the `platformio.ini` file to the USB device for your AmbaSat-1 satellite.


# Design
Below are the intended requirements for the finalized AmbaSat-1 flight software. This is very much a `Work in Progress`.

## Requirements

### Telemetry and Status Uplinks

#### Uplink Ports
The LoRaWAN protocol affords different communication ports to allow for transmitting different types of payloads. Each port shoul be used for a specific type of data packet to be uplinked. Each port should have a configurable downlink rate, and the satellite should have a maximum overall downlink rate (the sum of all port downlinks). The specific port mappings will be:

| Port | Description | Comments |
|:-:|:--|:--|
| 1 | Voltage Level | Also considered the "is alive" telemetry |
| 2 | LSM9DS1 | Acceleration, Rotation, and Magnetic Field |
| 3 | Sensor 1 | |
| 4 | Sensor 2 | |
| 5 | Sensor 3 | |
| 6 | Sensor 4 | |
| 7 | Sensor 5 | |
| 8 | Sensor 6 | |
| 9 | Sensor 7 | |
| 10 | Sensor 8 | |
| 11 | CMD Status | Report the status of a command request |

Since AmbaSat only supports one sensor other than the LSM9DS1, clearly not all of the ports will be in use when in flight. However, so that this software can be generally used, support for all sensor types will be created. In order to minimize the amount of space is required on the ATMEGA328, all sensor support will be configured with a compiler macro to either activate or deactivate the sensor specific code.

### Command Request Downlinks
In normal oprations, the AmbaSat is not intended to be regularly sent commands to change how it operates. However, the ability to alter operational paramters of the satellite will be enabled using the TTN downlink abilities. 

The command will consist of atleast 1 byte. This byte will be split into the high and low 4 bits. The high 4 bits will represent the actual command, and the low fit bits will represent the sensor ports the command pertains to, if relevant. As a result, there can only be 16 commands that the satellite will recognize, and there are only 15 sensor ports the satellite can support. A port value of 0 for a command indicates the command does not apply to a particular sensor. The command can have an optional second and third byte that will contain configuration data, depedending on the command. 

#### Set Sensor Telemtry Uplink Rate
All sensors other than Voltage Level shall have the ability to alter the rate at which their telemtry is gathered and transmitted. In one extreme, the sensor's telementry rate can be set to zero, which should be treated as equivalent to turning off the sensor. This might be needed if the sensor is malfunctioning, or it is desired to simply spend the daily airtime budget that TTN imposes differently between the sensors.

This command will have a second byte that represents the minimum number of 8 second multiples that the satellite should sleep in between sending the indicated sensor's data payload. A value of `0x00` means to stop transmitting the indicated sensor's measurements altogether. 

#### Reset Uplink Counter

#### Reset Downlink Counter

#### Sensor Sensitivity Adjustment

#### Blink LED
Because the aliens in space like blinking lights too. The command sensor port indicator is ignored, and a second byte is used to indicate the number of times the LED should be blinked. 

## Design Issues

### ABP
The flight software will use ABP in order to minimize the radio chatter by not requiring two way communication that OTAA requires. 

#### Frame Counter Saving
As solar panels will be it's only source of power, it is expected that the satellite will reboot aften as it tumbles through space and goes in and out of light. The challenge here is that for ABP, TTN enforce frame counter checks as a security measure, and whenever the satellite reboots, it's frame counter is otherwise reset to 0 in LMIC. One way to address this is to relax theframe counter requirements in the TTN settings for the device. TTN requests that this be done for development reasons only since it does pose a security risk. A more robust want to handle this is to save the frame counter value to the onboard flash memory.

### Flash Memory Usage
In an effort to keep the code footprint small, the following actions are taken:
* **Reducing the LMIC code footprint** - Since we are be using ABP activation,we don't need the LMIC code pertaining to OTAA. To effect this, the following compiler macros will be defined:
** `DISABLE_JOIN`
** `DISABLE_PING`
** `DISABLE_BEACONS`
** `DISABLE_MCMD_PING_SET`
** `DISABLE_MCMD_BCNI_ANS`
* **Wrap `Serial.print()` in a macro** - In order to remove serial prints that are only really intended for debugging, a macro should be used that will allow the print statements to be compiled out in the flight build.
* **Unused sensor code not compiled** - Though this software is intended to be general purpose accross all sensor types, only the configured sensor should compiled and linked when building. This will be accomplish through compiler macros.


### Radition Hardness
Given the space environment, I would expect bits to get flipped due to the radition. I supposed there isn't much we can do about this given the low cost nature of the AmbaSat. However, we can implement some simple data integretity checks. `TBD`

## References
The following articles and documents are useful when designing this software
* [TTN LoraWan Atmega32U4 based node – ABP version](https://primalcortex.wordpress.com/2017/10/31/ttnlorawan32u4node/)
* [Frame counters in ABP mode](https://forum.chirpstack.io/t/frame-counters-in-abp-mode/811)
* [Where in LMiC is the “fCnt” parameter stored?](https://www.thethingsnetwork.org/forum/t/where-in-lmic-is-the-fcnt-parameter-stored/3082)
* [Unit Tests on Platform IO](https://www.thingforward.io/techblog/2017-07-25-starting-embedded-testing-with-platformio.html)
* [Example of How to Write to Flash Memory](https://github.com/MCUdude/MiniCore/blob/master/avr/libraries/Optiboot_flasher/examples/SerialReadWrite/SerialReadWrite.ino)