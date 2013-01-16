#!/bin/sh
flash_erase /dev/mtd1 0 0
nandwrite -p /dev/mtd1 /boot/uImage
