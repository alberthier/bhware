#!/bin/sh
./01-setup-nand-uboot-config.py
./02-flash-uImage1.sh
./03-flash-uImage2.sh
./04-flash-nand.sh
ubiattach -m 3
mount -t ubifs /dev/ubi0_0 /mnt
if [ "$1" != "" ] 
then
    VALUE1=$(sudo cat /mnt/etc/network/interfaces | grep -n "eth0 inet" | sed 's:\([0-9]*\).*:\1:')
    if [ "$VALUE1" != "" ]
    then
        VALUE2=1
        VALUE3=$((VALUE1+VALUE2))
        sed -i ''$VALUE3' c\    address '$1'' /mnt/etc/network/interfaces
    fi
fi
sync
umount /mnt/
sleep 1
reboot
