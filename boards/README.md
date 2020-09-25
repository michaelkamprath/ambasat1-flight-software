# AmbaSat-1 Board Definitions
The JSON files in this directory define the board configuration that should be used with PlaformIO for the AmbaSat-1 picosat. The board definitions are:

* `AmbaSat-1` - This is the original board definition provided by the AmbaSat makers. This definition expect the ATMEGA328P  to be configured to us the internal clock at 1 Mhz.
* `AmbaSat-1b` - This board definition enables the use of the external 4 MHz resonator on the AmbaSat-1 picosat and a BOD of 1.8V. Note that you need to update the ATMEGA328P's fuses and boot loader to use this board definition. 