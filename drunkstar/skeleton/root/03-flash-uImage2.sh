#!/bin/sh
flash_erase /dev/mtd2 0 0
nandwrite -p /dev/mtd2 /boot/uImage
