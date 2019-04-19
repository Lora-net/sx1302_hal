#!/bin/sh

# This script is intended to be used on SX1302 CoreCell platform, it performs
# the following actions:
#       - export/unpexort GPIO23 and GPIO18 used to reset the SX1302 chip and to enable the LDOs
#
# Usage examples:
#       ./reset_lgw.sh stop
#       ./reset_lgw.sh start

# GPIO mapping has to be adapted with HW
#

IOT_SK_SX1302_RESET_PIN=23
IOT_SK_SX1302_POWER_EN_PIN=18

echo "Accessing CoreCellSX1302 reset pin through GPIO$IOT_SK_SX1302_RESET_PIN..."
echo "Accessing CoreCellSX1302 power enable pin through GPIO$IOT_SK_SX1302_POWER_EN_PIN..."

WAIT_GPIO() {
    sleep 0.1
}

iot_sk_init() {
    # setup GPIOs
    echo "$IOT_SK_SX1302_RESET_PIN" > /sys/class/gpio/export; WAIT_GPIO
    echo "$IOT_SK_SX1302_POWER_EN_PIN" > /sys/class/gpio/export; WAIT_GPIO

    # set GPIOs as output
    echo "out" > /sys/class/gpio/gpio$IOT_SK_SX1302_RESET_PIN/direction; WAIT_GPIO
    echo "out" > /sys/class/gpio/gpio$IOT_SK_SX1302_POWER_EN_PIN/direction; WAIT_GPIO

    # write output for SX1302 CoreCell power_enable and reset
    echo "1" > /sys/class/gpio/gpio$IOT_SK_SX1302_POWER_EN_PIN/value; WAIT_GPIO

    echo "1" > /sys/class/gpio/gpio$IOT_SK_SX1302_RESET_PIN/value; WAIT_GPIO
    echo "0" > /sys/class/gpio/gpio$IOT_SK_SX1302_RESET_PIN/value; WAIT_GPIO
}

iot_sk_term() {
    # cleanup all GPIOs
    if [ -d /sys/class/gpio/gpio$IOT_SK_SX1302_RESET_PIN ]
    then
        echo "$IOT_SK_SX1302_RESET_PIN" > /sys/class/gpio/unexport; WAIT_GPIO
    fi
    if [ -d /sys/class/gpio/gpio$IOT_SK_SX1302_POWER_EN_PIN ]
    then
        echo "$IOT_SK_SX1302_POWER_EN_PIN" > /sys/class/gpio/unexport; WAIT_GPIO
    fi
}

case "$1" in
    start)
    iot_sk_term
    iot_sk_init
    ;;
    stop)
    iot_sk_term
    ;;
    *)
    echo "Usage: $0 {start|stop}"
    exit 1
    ;;
esac

exit 0