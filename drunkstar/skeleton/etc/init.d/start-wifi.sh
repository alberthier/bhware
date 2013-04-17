#!/bin/sh

while true; do
    HAS_WIFI=`iwconfig wlan0 | grep "No such device"`
    if [ !$HAS_WIFI ]; then
        iwconfig wlan0 mode ad-hoc essid BHInterbot key 0011223344
        ip link set wlan0 up
        if [ `hostname` = "sheldon" ]; then
            ifconfig wlan0 192.168.3.10
        elif [ `hostname` = "leonard" ]; then
            ifconfig wlan0 192.168.3.11
        else
            ifconfig wlan0 192.168.3.12
        fi
        exit 0
    fi
done

