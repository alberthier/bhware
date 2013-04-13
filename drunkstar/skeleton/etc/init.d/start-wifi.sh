#!/bin/sh

for i in 1 2 3 4 5 6 7 8 9 10; do
    echo "Wifi configuration #$i"
    iwconfig wlan0 mode ad-hoc essid BHInterbot key 0011223344
    ip link set wlan0 up
    if [ `hostname` = "sheldon" ]; then
        ifconfig wlan0 192.168.3.10
    elif [ `hostname` = "leonard" ]; then
        ifconfig wlan0 192.168.3.11
    else
        ifconfig wlan0 192.168.3.12
    fi
    if ifconfig wlan0; then
        exit 0
    else
        sleep 1
    fi
done

