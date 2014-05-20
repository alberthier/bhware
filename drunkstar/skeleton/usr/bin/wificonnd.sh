#!/bin/sh

IFACE_NAME="wlan0"

while true; do
    is_wifi_configured=1

    ifconfig_stdout=`ifconfig ${IFACE_NAME}`
    if [ $? -eq 0 ]; then
        echo ${ifconfig_stdout} | grep "inet addr" > /dev/null
        is_wifi_configured=$?
    fi

    if [ ${is_wifi_configured} -ne 0 ]; then
        killall wpa_supplicant
        ifdown ${IFACE_NAME}
        ifup ${IFACE_NAME}
    fi

    sleep 2
done
