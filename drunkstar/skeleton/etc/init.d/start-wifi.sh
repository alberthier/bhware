#!/bin/sh

while true; do
    HAS_WIFI=`iwconfig wlan0 | grep "No such device"`
    if [ !$HAS_WIFI ]; then
        ifconfig wlan0 down
        iwconfig wlan0 down
        iwconfig wlan0 mode ad-hoc essid BHInterbot key off
        if [ `hostname` = "sheldon" ]; then
            ifconfig wlan0 192.168.3.10 netmask 255.255.255.0 up
        elif [ `hostname` = "leonard" ]; then
            ifconfig wlan0 192.168.3.11 netmask 255.255.255.0 up
        else
            ifconfig wlan0 192.168.3.12 netmask 255.255.255.0 up
        fi
        exit 0
    fi
done

