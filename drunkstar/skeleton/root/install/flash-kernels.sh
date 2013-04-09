#!/bin/sh
flash_erase /dev/mtd1 0 0
nandwrite -p /dev/mtd1 /boot/uImage
flash_erase /dev/mtd2 0 0
nandwrite -p /dev/mtd2 /boot/uImage
