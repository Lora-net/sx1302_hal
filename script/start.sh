#!/bin/bash

if [ $# -eq 0 ]; then # [ $# -eq 0 ] || [ "$@" == "" ]
  echo usage: ./start.sh gateway, argc1: gateway argcs ...
  exit -1
fi

resetGpio=218

echo reset sx1302 module...
sudo echo $resetGpio > /sys/class/gpio/export
sudo echo high > /sys/class/gpio/gpio$resetGpio/direction
delay 1
sudo echo low> /sys/class/gpio/gpio$resetGpio/direction

echo start app...

if [ "$1" == "gateway" ]; then
  ./lora_pkt_fwd -c global_conf.json
elif [ "$1" == "id" ]; then
  ./chip_id $2 $3
else
  ./$1 $2 $3
fi

