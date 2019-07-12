	  ______                              _
	 / _____)             _              | |
	( (____  _____ ____ _| |_ _____  ____| |__
	 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
	 _____) ) ____| | | || |_| ____( (___| | | |
	(______/|_____)_|_|_| \__)_____)\____)_| |_|
	  (C)2019 Semtech

Utility: Downlink server
========================

## 1. Introduction

This utility allows to send regular downlink requests to the packet forwarder
running on the gateway.

The downlinks are sent in 'immediate' mode, meaning that the SX1302 will send
the incoming packet over the air as soon as it receives it.

So, the net_downlink utility will construct a JSON 'txpk' object based on given
command line arguments, and send it on a UDP socket on the given port. Then the
packet forwarder receives it on its downlink socket, parses the JSON object to
build the packet buffer to be sent to the concentrator board.

This utility can be compiled and run on the gateway itself, or on a PC.

Optionally, the net_downlink utility can forward the received uplinks
(PUSH_DATA) to another UDP server, which can be useful for uplink Packet Error
Rate measurement while performing downlink testing (full-duplex testing etc...)

In can also be used as a UDP packet logger, logging all uplinks in a CSV file.

## 2. Dependencies

A packet forwarder must be running to receive downlink packets and send it to
the concentrator board.

## 3. Usage

### 3.1. Packet Forwarder configuration

The 'global_conf.json' file provided with the packet forwarder can be used, only
the 'server_address' must be set to 'localhost' if net_downlink is running on
the gateway itself, or set to the IP address of the PC on which the utility is
running.

### 3.2. Launching the packet forwarder

The packet forwarder has to be launched with the global_conf.json described in
3.1.

 `./lora_pkt_fwd -c global_conf.json`

### 3.3. Launching net_downlink

The net_downlink utility can be started with various command line arguments.

In order to get the available options, and some examples, run:

`./net_downlink -h`

To stop the application, press Ctrl+C.
