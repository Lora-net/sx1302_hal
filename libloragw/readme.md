	 / _____)             _              | |
	( (____  _____ ____ _| |_ _____  ____| |__
	 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
	 _____) ) ____| | | || |_| ____( (___| | | |
	(______/|_____)_|_|_| \__)_____)\____)_| |_|
	  (C)2020 Semtech

LoRa concentrator HAL user manual
=================================

## 1. Introduction

The LoRa concentrator Hardware Abstraction Layer is a C library that allow you
to use a Semtech concentrator chip through a reduced number of high level C
functions to configure the hardware, send and receive packets.

The Semtech LoRa concentrator is a digital multi-channel multi-standard packet
radio used to send and receive packets wirelessly using LoRa or FSK modulations.

## 2. Components of the library

The library is composed of the following modules:

1. abstraction layer
  * loragw_hal
  * loragw_reg
  * loragw_aux
  * loragw_cal
  * loragw_lbt
  * loragw_sx1302
  * loragw_sx1302_rx
  * loragw_sx1302_timestamp
  * loragw_sx125x
  * loragw_sx1250
  * loragw_sx1261

2. communication layer for sx1302
  * loragw_com
  * loragw_spi
  * loragw_usb

3. communication layer for sx1255/SX1257 radios
  * sx125x_com
  * sx125x_spi

4. communication layer for sx1250 radios
  * sx1250_com
  * sx1250_spi
  * sx1250_usb

5. communication layer for STM32 MCU (USB)
  * loragw_mcu

6. communication layer for sx1261 radio (LBT / Spectral Scan)
  * sx1261_com
  * sx1261_spi
  * sx1261_usb

7. peripherals
  * loragw_i2c
  * loragw_gps
  * loragw_stts751
  * loragw_ad5338r

The library also contains basic test programs to demonstrate code use and check
functionality.

### 2.1. loragw_hal

This is the main module and contains the high level functions to configure and
use the LoRa concentrator:

* lgw_board_setconf, to set the configuration of the concentrator
* lgw_rxrf_setconf, to set the configuration of the radio channels
* lgw_rxif_setconf, to set the configuration of the IF+modem channels
* lgw_txgain_setconf, to set the configuration of the concentrator gain table
* lgw_start, to apply the set configuration to the hardware and start it
* lgw_stop, to stop the hardware
* lgw_receive, to fetch packets if any was received
* lgw_send, to send a single packet (non-blocking, see warning in usage section)
* lgw_status, to check when a packet has effectively been sent
* lgw_get_trigcnt, to get the value of the sx1302 internal counter at last PPS
* lgw_get_instcnt, to get the value of the sx1302 internal counter
* lgw_get_eui, to get the sx1302 chip EUI
* lgw_get_temperature, to get the current temperature
* lgw_time_on_air, to get the Time On Air of a packet
* lgw_spectral_scan_start, to start scaning a particular channel
* lgw_spectral_scan_get_status, to get the status of the current scan
* lgw_spectral_scan_get_results, to get the results of the completed scan
* lgw_spectral_scan_abort, to abort curretn scan

For an standard application, include only this module.
The use of this module is detailed on the usage section.

/!\ When sending a packet, there is a delay (approx 1.5ms) for the analog
circuitry to start and be stable. This delay is adjusted by the HAL depending
on the board version (lgw_i_tx_start_delay_us).

In 'timestamp' mode, this is transparent: the modem is started
lgw_i_tx_start_delay_us microseconds before the user-set timestamp value is
reached, the preamble of the packet start right when the internal timestamp
counter reach target value.

In 'immediate' mode, the packet is emitted as soon as possible: transferring the
packet (and its parameters) from the host to the concentrator takes some time,
then there is the lgw_i_tx_start_delay_us, then the packet is emitted.

In 'triggered' mode (aka PPS/GPS mode), the packet, typically a beacon, is
emitted lgw_i_tx_start_delay_us microsenconds after a rising edge of the
trigger signal. Because there is no way to anticipate the triggering event and
start the analog circuitry beforehand, that delay must be taken into account in
the protocol.

### 2.2. loragw_reg

This module is used to access to the LoRa concentrator registers by name instead
of by address:

* lgw_connect, to initialise and check the connection with the hardware
* lgw_disconnect, to disconnect the hardware
* lgw_reg_r, read a named register
* lgw_reg_w, write a named register
* lgw_reg_rb, read a name register in burst
* lgw_reg_wb, write a named register in burst
* lgw_mem_rb, read from a memory section in burst
* lgw_mem_wb, write to a memory section in burst

This module handles read-only registers protection, multi-byte registers
management, signed registers management, read-modify-write routines for
sub-byte registers and read/write burst fragmentation to respect SPI/USB maximum
burst length constraints.

It make the code much easier to read and to debug.
Moreover, if registers are relocated between different hardware revisions but
keep the same function, the code written using register names can be reused "as
is".

If you need access to all the registers, include this module in your
application.

**/!\ Warning** please be sure to have a good understanding of the LoRa
concentrator inner working before accessing the internal registers directly.

### 2.3. loragw_com

This module contains the functions to access the LoRa concentrator register
array through the SPI or USB interfaces:

* lgw_com_r to read one byte
* lgw_com_w to write one byte
* lgw_com_rb to read two bytes or more
* lgw_com_wb to write two bytes or more

This modules is an abstract interface, it then relies on the following modules
to actually perform the interfacing:

* loragw_spi : for SPI interface
* loragw_usb : for USB interface

Please *do not* include that module directly into your application.

**/!\ Warning** Accessing the LoRa concentrator register array without the
checks and safety provided by the functions in loragw_reg is not recommended.

### 2.4. loragw_aux

This module contains a single host-dependant function wait_ms to pause for a
defined amount of milliseconds.

The procedure to start and configure the LoRa concentrator hardware contained in
the loragw_hal module requires to wait for several milliseconds at certain
steps, typically to allow for supply voltages or clocks to stabilize after been
switched on.

An accuracy of 1 ms or less is ideal.
If your system does not allow that level of accuracy, make sure that the actual
delay is *longer* that the time specified when the function is called (ie.
wait_ms(X) **MUST NOT** before X milliseconds under any circumstance).

If the minimum delays are not guaranteed during the configuration and start
procedure, the hardware might not work at nominal performance.
Most likely, it will not work at all.

### 2.5. loragw_gps

This module contains functions to synchronize the concentrator internal
counter with an absolute time reference, in our case a GPS satellite receiver.

The internal concentrator counter is used to timestamp incoming packets and to
triggers outgoing packets with a microsecond accuracy.
In some cases, it might be useful to be able to transform that internal
timestamp (that is independent for each concentrator running in a typical
networked system) into an absolute GPS time.

In a typical implementation a GPS specific thread will be called, doing the
following things after opening the serial port:

* blocking reads on the serial port (using system read() function)
* parse UBX messages (using lgw_parse_ubx) to get actual native GPS time
* parse NMEA sentences (using lgw_parse_nmea) to get location and UTC time
Note: the RMC sentence gives UTC time, not native GPS time.

And each time an NAV-TIMEGPS UBX message has been received:

* get the concentrator timestamp (using lgw_get_trigcnt, mutex needed to
  protect access to the concentrator)
* get the GPS time contained in the UBX message (using lgw_gps_get)
* call the lgw_gps_sync function (use mutex to protect the time reference that
  should be a global shared variable).

Then, in other threads, you can simply used that continuously adjusted time
reference to convert internal timestamps to GPS time (using lgw_cnt2gps) or
the other way around (using lgw_gps2cnt). Inernal concentrator timestamp can
also be converted to/from UTC time using lgw_cnt2utc/lgw_utc2cnt functions.

### 2.6. loragw_sx125x

This module contains functions to handle the configuration of SX1255 and
SX1257 radios. In order to communicate with the radio, it relies on the
following modules:

* sx125x_com : abstract interfacing to select USB or SPI interface
* sx125x_spi : implementation of the SPI interface

### 2.7. loragw_sx1250

This module contains functions to handle the configuration of SX1250 radios. In
order to communicate with the radio, it relies on the following modules:

* sx1250_com : abstract interfacing to select USB or SPI interface
* sx1250_spi : implementation of the SPI interface
* sx1250_usb : implementation of the USB interface

### 2.8. loragw_sx1302

This module contains functions to abstract SX1302 concentrator capabilities.

### 2.9. loragw_sx1302_rx

This module is a sub-module of the loragw_sx1302 module focusing on abstracting
the RX buffer of the SX1302.

### 2.10. loragw_sx1302_timestamp

This module is a sub-module of the loragw_sx1302 module focusing on abstracting
the timestamp counter of the SX1302.
It converts the 32-bits 32MHz internal counter of the SX1302 to a 32-bits 1MHz
counter.
This module needs to be called regularly by upper layers to maintain counter
wrapping when converting from 32MHz to 1MHz.
It also provides function to add correction to the timestamp counter to take
into account the LoRa demodulation processing time.

### 2.11. loragw_stts751

This module contains a very basic driver for the STmicroelectronics ST751
temperature sensor which is on the CoreCell reference design.

### 2.12. loragw_ad5338r

This module contains a very basic driver for the Analog Devices AD5338R DAC used
on the Semtech CN490 Full Duplex reference design to set the PA fixed gain.

### 2.13. loragw_i2c

This module provides basic function to communicate with I2C devices on the board.
It is used in this project for accessing the temperature sensor, the AD5338R DAC...

### 2.14. loragw_sx1261

This module contains functions to handle the configuration of SX1261 radio for
Listen-Before-Talk or Spectral Scan functionnalities. In order to communicate
with the radio, it relies on the following modules:

* sx1261_com : abstract interfacing to select USB or SPI interface
* sx1261_spi : implementation of the SPI interface
* sx1261_usb : implementation of the USB interface

This module will also load the sx1261 firmware patch RAM, necessary to support
Listen-Before-Talk and spectral scan features, from the sx1261_pram.var file.

### 2.15. loragw_lbt

This module contains functions to start and stop the Listen-Before-Talk feature
when it is enabled. Those functions are called by the lgw_send() function to
ensure that the concentrator is allowed to transmit.

Listen-Before-Talk (LBT) and Spectral Scan features need an additional sx1261
radio to be configured.

The Listen-Before-Talk feature works as follows:

* the HAL configures the sx1261 for scanning the channel on which it needs to
transmit.
* the SX1261 will scan the channel and set a GPIO to high or low depending if
the channel is busy or not (according to scanning parameters)
* the sx1302 AGC firmware will check the status of this GPIO before actually
starting the transmit, to ensure it is allowed. The AGC fw sets its status
register to inform if the transmit could be done or not.
* the HAL waits for the transmit to be initiated and checks if it was allowed or
not.
* the HAL stops the scanning, and return the tramsit status to the caller.

### 2.16. loragw_mcu

This module contains the functions to setup the communication interface with the
STM32 MCU, and to communicate with the sx1302 and the radios when the host and
the concentrator are connected through USB llink.

The MCU acts as a simple USB <-> SPI bridge. This means that the HAL running on
the host is the same, for both SPI or USB gateways.

But, as the USB communication link brings a 1ms latency for each transfer, the
MCU provides a mean to group register write requests in one single USB transfer.
It is necessary when a particular configuration has to be done in a time
critical task.

For this, 2 new functions has been added:
* lgw_com_set_write_mode, to indicate if the following calls to lgw_com_w(b)
need to be grouped on a single USB transfer (BULK mode) or not (SINGLE mode).
* lgw_com_flush, to actually perform the USB transfer of all grouped commands
if BULK mode was selected.

Both functions will do nothing in case of SPI.

The same mechanism can be used to configure the sx1261 radio.

## 3. Software build process

### 3.1. Details of the software

The library is written following ANSI C conventions but using C99 explicit
length data type for all data exchanges with hardware and for parameters.

The loragw_aux module contains POSIX dependant functions for millisecond
accuracy pause.
For embedded platforms, the function could be rewritten using hardware timers.

### 3.2. Building options

All modules use a fprintf(stderr,...) function to display debug diagnostic
messages if the DEBUG_xxx is set to 1 in library.cfg

### 3.3. Building procedures

For cross-compilation set the ARCH and CROSS_COMPILE variables in the Makefile,
or in your shell environment, with the correct toolchain name and path.
ex:
export PATH=/home/foo/rpi-toolchain/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin:$PATH
export ARCH=arm
export CROSS_COMPILE=arm-linux-gnueabihf-

The Makefile in the libloragw directory will parse the library.cfg file and
generate a config.h C header file containing #define options.
Those options enables and disables sections of code in the loragw_xxx.h files
and the *.c source files.

The library.cfg is also used directly to select the proper set of dynamic
libraries to be linked with.

### 3.4. Export

Once build, to use that library on another system, you need to export the
following files :

* libloragw/library.cfg  -> root configuration file
* libloragw/libloragw.a  -> static library, to be linked with a program
* libloragw/readme.md  -> required for license compliance
* libloragw/inc/config.h  -> C configuration flags, derived from library.cfg
* libloragw/inc/loragw_*.h  -> take only the ones you need (eg. _hal and _gps)

After statically linking the library to your application, only the license
is required to be kept or copied inside your program documentation.

## 4. Hardware dependencies

### 4.1. Hardware revision

The loragw_reg and loragw_hal are written for a specific version on the Semtech
hardware (IP and/or silicon revision).

This code has been written for:

* Semtech SX1302 chip
* Semtech SX1250, SX1257 or SX1255 I/Q transceivers

The library will not work if there is a mismatch between the hardware version
and the library version. You can use the test program test_loragw_reg to check
if the hardware registers match their software declaration.

### 4.2. GPS receiver (or other GNSS system)

To use the GPS module of the library, the host must be connected to a GPS
receiver via a serial link (or an equivalent receiver using a different
satellite constellation).
The serial link must appear as a "tty" device in the /dev/ directory, and the
user launching the program must have the proper system rights to read and
write on that device.
Use `chmod a+rw` to allow all users to access that specific tty device, or use
sudo to run all your programs (eg. `sudo ./test_loragw_gps`).

In the current revision, the library only reads data from the serial port,
expecting to receive NMEA frames that are generally sent by GPS receivers as
soon as they are powered up, and UBX messages which are proprietary to u-blox
modules.

The GPS receiver **MUST** send UBX messages shortly after sending a PPS pulse
on to allow internal concentrator timestamps to be converted to absolute GPS time.
If the GPS receiver sends a GGA NMEA sentence, the gateway 3D position will
also be available.

### 4.3. Additionnal SX1261 radio

In order to perform Listen-Before-Talk and/or Spectral Scan, an additional SX1261
radio is required. Its internal firmware also needs to be patched (patch RAM) to
support those particular features.

## 5. Usage

### 5.1. Setting the software environment

For a typical application you need to:

* include loragw_hal.h in your program source
* link to the libloragw.a static library during compilation
* link to the librt library due to loragw_aux dependencies (timing functions)

For an application that will also access the concentrator configuration
registers directly (eg. for advanced configuration) you also need to:

* include loragw_reg.h in your program source

### 5.2. Using the software API

To use the HAL in your application, you must follow some basic rules:

* configure the radios path and IF+modem path before starting the radio
* the configuration is only transferred to hardware when you call the *start*
  function
* you cannot receive packets until one (or +) radio is enabled AND one (or +)
  IF+modem part is enabled AND the concentrator is started
* you cannot send packets until one (or +) radio is enabled AND the concentrator
  is started
* you must stop the concentrator before changing the configuration

A typical application flow for using the HAL is the following:

	<configure the radios and IF+modems>
	<start the LoRa concentrator>
	loop {
		<fetch packets that were received by the concentrator>
		<process, store and/or forward received packets>
		<send packets through the concentrator>
	}
	<stop the concentrator>

**/!\ Warning** The lgw_send function is non-blocking and returns while the
LoRa concentrator is still sending the packet, or even before the packet has
started to be transmitted if the packet is triggered on a future event.
While a packet is emitted, no packet can be received (limitation intrinsic to
most radio frequency systems).

Your application *must* take into account the time it takes to send a packet or
check the status (using lgw_status) before attempting to send another packet.

Trying to send a packet while the previous packet has not finished being send
will result in the previous packet not being sent or being sent only partially
(resulting in a CRC error in the receiver).

### 5.3. Debugging mode

To debug your application, it might help to compile the loragw_hal function
with the debug messages activated (set DEBUG_HAL=1 in library.cfg).
It then send a lot of details, including detailed error messages to *stderr*.

## 6. Notes

### 6.1. Spreading factor SF5 & SF6

The sx1302 supports SF5 and SF6 spreading factors, and the HAL also. But it is
important to note that the only syncword supported for SF5 and SF6 is 0x12
(also known as "private").

This is true whatever how of the "lorawan_public" field of lgw_conf_board_s is
set.

## 7. License

Copyright (c) 2019, SEMTECH S.A.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.
* Neither the name of the Semtech corporation nor the
  names of its contributors may be used to endorse or promote products
  derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL SEMTECH S.A. BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*EOF*
