	  ______                              _
	 / _____)             _              | |
	( (____  _____ ____ _| |_ _____  ____| |__
	 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
	 _____) ) ____| | | || |_| ____( (___| | | |
	(______/|_____)_|_|_| \__)_____)\____)_| |_|
	  (C)2014 Semtech-Cycleo

Utility: Downlink server
========================


## 1. Introduction
------------------

This utility allows to send regular downlink requests to the packet forwarder
running on the gateway.

The downlinks are sent in 'immediate' mode, meaning that the SX1301 will send
the incoming packet over the air as soon as it receives it.

So, the net_downlink utility will construct a JSON 'txpk' object based on given
command line arguments, and send it on a UDP socket on the given port. Then the
packet forwarder receives it on its downlink socket, parses the JSON object to
build the packet buffer to be sent to the concentrator board.

This utility can be compiled and run on the gateway itself, or on a PC.

Optionally, the net_downlink utility can forward the received uplinks
(PUSH_DATA) to another UDP server, which can be useful for uplink Packet Error
Rate measurement while performing downlink testing (full-duplex testing etc...)


## 2. Dependencies
------------------

A packet forwarder must be running to receive downlink packets and send it to
the concentrator board.


## 3. Usage
-----------

### 3.1. Packet Forwarder configuration

The 'config.json' file provided with the packet forwarder can be used, only the
'server_address' must be set to 'localhost' if net_downlink is running on the
gateway itself, or set to the IP address of the PC on which the utility is
running.

If needed, a pure downlink configuration can be used with:
- 'rx1_enable' and 'rx2_enable' set to 'false'.
- 'tx1_enable' and 'tx2_enable' set to 'true'.
- 'serv_port_up' set to '0' or to anything else than serv_port_down.
- 'serv_port_down' set to '1690', but can be changed.

### 3.2. Launching the packet forwarder

The packet forwarder has to be launched with the config.json described in 3.1.,
and with GPS disabled.

 ./pkt_forwarder -c config.json

### 3.3. Launching net_downlink

The net_downlink utility can be started with various command line arguments
described below:

#### 3.3.1. General options

'-h'
will display a short help

'x'
will set the number of downlink packets to be sent

't'
will set the delay in milliseconds between 2 consecutive downlinks

'-P'
will set the UDP port on which to sent the packet. It should match the
'serv_port_down' defined in the 'config.json' used by packet forwarder.

'-A'
will set the IP address of the UDP server to which net_downlink has to forward
the received uplink messages (PUSH_DATA).

'-F'
will set the UDP port on which net_downlink has to forward the the received
uplink messages (PUSH_DATA).

#### 3.3.2. LoRa packet description options

'-f'
will set the target frequency in MHz [863..870MHz]

'-a'
will set the radio chain to be used for sending the packet [0, 1]

'-b'
will set the LoRa bandwidth in kHz [125, 250, 500]

'-s'
will set the LoRa Spreading Factor [7-12]

'-c'
will set the LoRa Coding Rate ["4/5", "4/6", ...]

'-p'
will set the TX RF power (dBm). The TX Gain table used to configure the
concentrator board is described in the 'config.json' file ('tx_lut_X').

'-z'
will set the size of the downlink packet payload (bytes, <256)

#### 3.3.3. Examples

* sending regular downlink requests to the packet forwarder
 ./net_downlink -f 869.525 -a 0 -s 9 -b 125 -c "4/5" -p 20 -z 8 -P 1690 -x 200 -t 1000

* sending regular downlink requests to the packet forwarder and forwarding uplinks to another UDP server
 ./net_downlink -f 869.525 -a 0 -s 9 -b 125 -c "4/5" -p 20 -z 8 -P 1690 -x 200 -t 1000 -A 52.18.106.103 -F 20000

To stop the application, press Ctrl+C.
