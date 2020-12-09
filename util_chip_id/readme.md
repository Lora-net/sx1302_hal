	  ______                              _
	 / _____)             _              | |
	( (____  _____ ____ _| |_ _____  ____| |__
	 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
	 _____) ) ____| | | || |_| ____( (___| | | |
	(______/|_____)_|_|_| \__)_____)\____)_| |_|
	  (C)2020 Semtech

Utility to get SX1302 chip EUI
==============================


## 1. Introduction

This utility configures the SX1302 to be able to retrieve its EUI.
It can then be used as a Gateway ID.

## 2. Command line options

### 2.1. General options ###

`-h`
will display a short help and version informations.

### 2.2. SPI options ###

`-d spidev_path`
use the Linux SPI device driver, but with an explicit path, for systems with
several SPI device drivers, or uncommon numbering scheme.

### 2.3. USB options ###

`-u -d tty_path`
use the TTY path associated with the gateway.

## 3. Legal notice

The information presented in this project documentation does not form part of
any quotation or contract, is believed to be accurate and reliable and may be
changed without notice. No liability will be accepted by the publisher for any
consequence of its use. Publication thereof does not convey nor imply any
license under patent or other industrial or intellectual property rights.
Semtech assumes no responsibility or liability whatsoever for any failure or
unexpected operation resulting from misuse, neglect improper installation,
repair or improper handling or unusual physical or electrical stress
including, but not limited to, exposure to parameters beyond the specified
maximum ratings or operation outside the specified range.

SEMTECH PRODUCTS ARE NOT DESIGNED, INTENDED, AUTHORIZED OR WARRANTED TO BE
SUITABLE FOR USE IN LIFE-SUPPORT APPLICATIONS, DEVICES OR SYSTEMS OR OTHER
CRITICAL APPLICATIONS. INCLUSION OF SEMTECH PRODUCTS IN SUCH APPLICATIONS IS
UNDERSTOOD TO BE UNDERTAKEN SOLELY AT THE CUSTOMER'S OWN RISK. Should a
customer purchase or use Semtech products for any such unauthorized
application, the customer shall indemnify and hold Semtech and its officers,
employees, subsidiaries, affiliates, and distributors harmless against all
claims, costs damages and attorney fees which could arise.

*EOF*