#!/usr/bin/python


import sys
from subprocess import *


def clear():
    for line in check_output(["fw_printenv"]).splitlines():
        i = line.find("=")
        key = line[:i].strip()
        value = line[i+1:].strip()
        if key != "ethaddr":
            call(["fw_setenv", key])


def setup():
    if len(sys.argv) > 1 and sys.argv[1] == "nand":
        call(["fw_setenv", "bootcmd"        , "run bootcmd_nand; run bootcmd_usb; usb stop; reset"])
    else:
        call(["fw_setenv", "bootcmd"        , "run bootcmd_usb; usb stop; run bootcmd_nand; reset"])
    call(["fw_setenv", "baudrate"           , "115200"])
    call(["fw_setenv", "arcNumber"          , "2998"])
    call(["fw_setenv", "mainlineLinux"      , "yes"])
    call(["fw_setenv", "console"            , "ttyS0,115200"])
    call(["fw_setenv", "common_bootargs"    , "mtdparts=orion_nand:1M(u-boot),3M(uImage1),3M(uImage2),-(data) lpj=5955584"])
    call(["fw_setenv", "mtdids"             , "nand0=orion_nand"])
    call(["fw_setenv", "partition"          , "nand0,4"])
    call(["fw_setenv", "usb_init"           , "usb start"])
    call(["fw_setenv", "usb_device"         , "0:1"])
    call(["fw_setenv", "usb_root"           , "/dev/sda2"])
    call(["fw_setenv", "usb_rootfstype"     , "ext4"])
    call(["fw_setenv", "usb_rootdelay"      , "3"])
    call(["fw_setenv", "usb_load_uimage"    , "mw 0x800000 0 1; ext2load usb $usb_device 0x800000 /boot/uImage"])
    call(["fw_setenv", "usb_boot"           , "if ext2load usb $usb_device 0x1100000 /boot/uInitrd; then bootm 0x800000 0x1100000; else bootm 0x800000; fi;"])
    call(["fw_setenv", "bootcmd_usb"        , "run usb_init; run usb_load_uimage; run set_bootargs_usb; run usb_boot;"])
    call(["fw_setenv", "set_bootargs_usb"   , "setenv bootargs console=$console root=$usb_root rootdelay=$usb_rootdelay rootfstype=$usb_rootfstype $common_bootargs"])
    call(["fw_setenv", "ubi_part"           , "data"])
    call(["fw_setenv", "ubi_volume"         , "rootfs"])
    call(["fw_setenv", "preboot"            , "run if_netconsole start_netconsole"])
    call(["fw_setenv", "set_bootargs_nand"  , "setenv bootargs console=$console ubi.mtd=$ubi_part root=ubi0:$ubi_volume rootfstype=ubifs $common_bootargs"])
    call(["fw_setenv", "led_init"           , "green blinking"])
    call(["fw_setenv", "led_exit"           , "green off"])
    call(["fw_setenv", "led_error"          , "orange blinking"])
    call(["fw_setenv", "stdin"              , "serial"])
    call(["fw_setenv", "stdout"             , "serial"])
    call(["fw_setenv", "stderr"             , "serial"])
    call(["fw_setenv", "ethact"             , "egiga0"])
    call(["fw_setenv", "serverip"           , "192.168.0.2"])
    call(["fw_setenv", "ipaddr"             , "192.168.0.42"])
    call(["fw_setenv", "if_netconsole"      , "ping $serverip"])
    call(["fw_setenv", "start_netconsole"   , "setenv ncip $serverip; setenv stdin nc; setenv stdout nc; setenv stderr nc; version;"])
    call(["fw_setenv", "read_uImage1"       , "nand read.e 0x800000 0x100000 0x300000"])
    call(["fw_setenv", "read_uImage2"       , "nand read.e 0x800000 0x400000 0x300000"])
    call(["fw_setenv", "bootcmd_nand"       , "run set_bootargs_nand; run read_uImage1; bootm 0x800000; run read_uImage2; bootm 0x800000"])
    call(["fw_setenv", "bootdelay"          , "1"])
    call(["fw_printenv"])


if __name__ == "__main__":
    clear()
    setup()

