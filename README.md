# AmbaSat-1 Flight Software
This repository contains a complete, integrated suite of flight software for the [AmbaSat-1 picosat](https://ambasat.com). This software is not officially associated with the AmbaSat-1 makers, but instead the work of Michael Kamprath, a participant in the AmbaSat-1 project. This software is being released as open source in the hopes that other participants in the AmbaSat-1 project find it useful

## Project Structure
This project has the following main areas:
* `satellite-software` - This directory contains the software that gets flashed onto the AmbaSat-1 picosat. This directory is a [PlatformIO](https://platformio.org) project, set up to be used in [Visual Studio Code](https://code.visualstudio.com). The README in this directory gives comprehensive requirements for what the satellite will be able to do with this software, and by extension, how the ground software will support it.
* `ground-software` - This directory contains several pieces of software that will be used on the ground (not in the satellite). Most notable is the payload decoder for the telemetry sent to [The Things Network](https://www.thethingsnetwork.org).
* `boards` - These are the PlatformIO board definitions for the AmbaSat-1 hardware. 

## Contributing
Pull requests are welcome! However, please do read and understand the `satellite-software` requirements first and keep any changes in line with the intentions of those requirements. 

This software has been licensed under the GPL v3 license.

## Notes
This project is very much a work in progress. Please expect things to change significantly over time.