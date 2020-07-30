# Documentation for Serial Squarewave Chirp

## Usage

### Setup with Simplicity

This code is meant to be run on a Silicon Labs EFR32BG22, Blue Gecko board. Code provided from Silicon Labs (such as the EM Library) is not included in this project.

### Board Setup

This project relies on a piezo speaker soldered between pins 15 and 1, in parallel with a zener diode for protection. A MicroUSB cable should be connecting the board to your computer, we will use this connection as a serial port for data logging.

## Data Logging

To record data from your device, use the python script read.py, or your serial terminal of choice (PuTTY works well for this).

Certain characters trigger certain actions from the device.
 * r - record data
 * t - transmit data
