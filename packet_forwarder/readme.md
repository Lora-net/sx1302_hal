	 / _____)             _              | |
	( (____  _____ ____ _| |_ _____  ____| |__
	 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
	 _____) ) ____| | | || |_| ____( (___| | | |
	(______/|_____)_|_|_| \__)_____)\____)_| |_|
	  (C)2019 Semtech

SX1302 packet forwarder
=======================

## 1. Introduction

The packet forwarder is a program running on the host of a Lora gateway that
forwards RF packets receive by the concentrator to a server through a IP/UDP
link, and emits RF packets that are sent by the server. It can also emit a
network-wide GPS-synchronous beacon signal used for coordinating all nodes of
the network.

To learn more about the network protocol between the gateway and the server,
please read the PROTOCOL.TXT document.

## 2. System schematic and definitions

	((( Y )))
	    |
	    |
	+- -|- - - - - - - - - - - - -+        xxxxxxxxxxxx          +--------+
	|+--+-----------+     +------+|       xx x  x     xxx        |        |
	||              |     |      ||      xx  Internet  xx        |        |
	|| Concentrator |<----+ Host |<------xx     or    xx-------->|        |
	||              | SPI |      ||      xx  Intranet  xx        | Server |
	|+--------------+     +------+|       xxxx   x   xxxx        |        |
	|   ^                    ^    |           xxxxxxxx           |        |
	|   | PPS  +-----+  NMEA |    |                              |        |
	|   +------| GPS |-------+    |                              +--------+
	|          +-----+            |
	|                             |
	|            Gateway          |
	+- - - - - - - - - - - - - - -+

Concentrator: radio RX/TX board, based on Semtech multichannel modems (SX130x),
transceivers (SX135x) and/or low-power stand-alone modems (SX127x).

Host: embedded computer on which the packet forwarder is run. Drives the
concentrator through a SPI link.

Gateway: a device composed of at least one radio concentrator, a host, some
network connection to the internet or a private network (Ethernet, 3G, Wifi,
microwave link), and optionally a GPS receiver for synchronization.

Server: an abstract computer that will process the RF packets received and
forwarded by the gateway, and issue RF packets in response that the gateway
will have to emit.


## 3. Dependencies

This program uses the Parson library (http://kgabis.github.com/parson/) by
Krzysztof Gabis for JSON parsing.
Many thanks to him for that very practical and well written library.

This program is statically linked with the libloragw Lora concentrator library.
It was tested with v1.3.0 of the library but should work with any later
version provided the API is v1 or a later backward-compatible API.
Data structures of the received packets are accessed by name (ie. not at a
binary level) so new functionalities can be added to the API without affecting
that program at all.

This program follows the v1.3 version of the gateway-to-server protocol.

The last dependency is the hardware concentrator (based on FPGA or SX130x
chips) that must be matched with the proper version of the HAL.

## 4. Usage

* Pick the global_conf.json file that fit with your platform, region and feature
need.
* Update the JSON configuration (global and local) files, as explained below.
* Run:
    ./lora_pkt_fwd -c global_conf.json

To stop the application, press Ctrl+C.
Unless it is manually stopped or encounter a critical error, the program will
run forever.

The way the program takes configuration files into account is the following:
 * if a specific configuration file is given as program argument, use the
  one specified.
 * if there is a global_conf.json parse it.

The global configuration file should be exactly the same throughout your
network, contain all global parameters (parameters for "sensor" radio
channels) and preferably default "safe" values for parameters that are
specific for each gateway (eg. specify a default MAC address).

In each configuration file, the program looks for a JSON object named
"SX130x_conf" that should contain the parameters for the Lora concentrator
board (RF channels definition, modem parameters, etc) and another JSON object
called "gateway_conf" that should contain the gateway parameters (gateway MAC
address, IP address of the server, keep-alive time, etc).

To learn more about the JSON configuration format, read the provided JSON
files and the libloragw API documentation.

Every X seconds (parameter settable in the configuration files) the program
display statistics on the RF packets received and sent, and the network
datagrams received and sent.
The program also send some statistics to the server in JSON format.

## 5. "Just-In-Time" downlink scheduling

The LoRa concentrator can have only one TX packet programmed for departure at a
time. The actual departure of a downlink packet will be done based on its
timestamp, when the concentrator internal counter reaches timestamp’s value.
The departure of a beacon will be done based on a GPS PPS event.
It may happen that, due to network variable latency, the gateway receives one
or many downlink packets from the server while a TX is already programmed in the
concentrator. The packet forwarder has to store and order incoming downlink
packets (in a queue), so that they can all be programmed in the concentrator at
the proper time and sent over the air.
Possible failures that may occur and that have to be reported to the server are:
- It is too early or too late to send a given packet
- A packet collides with another packet already queued
- A packet collides with a beacon
- TX RF parameters (frequency, power) are not supported by gateway
- Gateway’s GPS is unlocked, so cannot process Class B downlink
It is called "Just-in-Time" (JiT) scheduling, because the packet forwarder will
program a downlink or a beacon packet in the concentrator just before it has to
be sent over the air.
Another benefit of JiT is to optimize the gateway downlink capacity by avoiding
to keep the concentrator TX buffer busy for too long.

In order to achieve "Just-in-Time" scheduling, the following software elements
have been added:
- A JiT queue, with associated enqueue/peek/dequeue functions and packet
acceptance criterias. It is where downlink packets are stored, waiting to be
sent.
- A JiT thread, which regularly checks if there is a packet in the JiT queue
ready to be programmed in the concentrator, based on current concentrator
internal time.

### 5.1. Concentrator vs GPS time synchronization

There are 2 cases for which we need to convert a GPS time to concentrator
counter:
    - Class B downlink: when the “time” field of JSON “txpk” is filled instead
      of the “tmst” field, we need to be able to determine if the packet can be
      queued in JiT queue or not, based on its corresponding concentrator
      counter value.
      Note: even if a Class-B downlink is given with a GPS timestamp, the
      concentrator TX mode is configured as “TIMESTAMP”, and not “ON_GPS”. So
      at the end, it is the counter value which will be used for transmission.
    - Beacons: beacons transmission time is based on GPS clock, and the
      concentrator TX mode is configured as “ON_GPS” for accurate beacon
      transmission on GPS PPS event. In this case, the concentrator does not
      need the packet counter to be set. But, as the JiT thread decides if a
      packet has to be peeked or not, based on its concentrator counter, we need
      to have the beacon packet counter set (see next chapter for more details
      on JiT scheduling).
We also need to convert a SX1302 counter value to GPS UTC time when we receive
an uplink, in order to fill the “time” field of JSON “rxpk” structure.

### 5.2. TX scheduling

The JiT queue implemented is a static array of nodes, where each node contains:
    - the downlink packet, with its type (beacon, downlink class A, B or C)
    - a “pre delay” which depends on packet type (BEACON_GUARD, TX_START_DELAY…)
    - a “post delay” which depends on packet type (“time on air” of this packet
      computed based on its size, datarate and coderate, or BEACON_RESERVED)

Several functions are implemented to manipulate this queue or get info from it:
    - init: initialize array with default values
    - is full / is empty: gives queue status
    - enqueue: checks if the given packet can be queued or not, based on several
      criteria’s
    - peek: checks if the queue contains a packet that must be passed
      immediately to the concentrator for transmission and returns corresponding
      index if any.
    - dequeue: actually removes from the queue the packet at index given by peek
      function

The queue is always kept sorted on ascending timestamp order.

The JiT thread will regularly check in the JiT queue if there is a packet to be
sent soon.  If a packet is matching, it is dequeued and programmed in the
concentrator TX buffer.

### 5.3. Fine tuning parameters

There are few parameters of the JiT queue which could be tweaked to adapt to
different system constraints.

    - inc/jitqueue.h:
        JIT_QUEUE_MAX: The maximum number of nodes in the queue.
    - src/jitqueue.c:
        TX_JIT_DELAY: The number of milliseconds a packet is programmed in the
                      concentrator TX buffer before its actual departure time.
        TX_MARGIN_DELAY: Packet collision check margin

### 6. License

Copyright (C) 2019, SEMTECH S.A.
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

#### 7. License for Parson library

Parson ( http://kgabis.github.com/parson/ )
Copyright (C) 2012 Krzysztof Gabis

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*EOF*
