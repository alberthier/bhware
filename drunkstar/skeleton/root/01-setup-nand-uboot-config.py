#!/usr/bin/python


from subprocess import *


def clear():
    for line in check_output(["fw_printenv"]).splitlines():
        i = line.find("=")
        key = line[:i].strip()
        value = line[i+1:].strip()
        if key != "ethaddr":
            call(["fw_setenv", key])

def setup():
    call(["fw_setenv", "bootcmd"          , "run bootcmd_nand; run bootcmd_usb; usb stop; reset"])
    call(["fw_setenv", "baudrate"         , "115200"])
    call(["fw_setenv", "arcNumber"        , "2097"])
    call(["fw_setenv", "mainlineLinux"    , "yes"])
    call(["fw_setenv", "console"          , "ttyS0,115200"])
    call(["fw_setenv", "mtdparts"         , "mtdparts=orion_nand:1M(u-boot),3M(uImage1),3M(uImage2),-(data)"])
    call(["fw_setenv", "mtdids"           , "nand0=orion_nand"])
    call(["fw_setenv", "partition"        , "nand0,4"])
    call(["fw_setenv", "usb_init"         , "usb start"])
    call(["fw_setenv", "usb_device"       , "0:1"])
    call(["fw_setenv", "usb_root"         , "/dev/sda1"])
    call(["fw_setenv", "usb_rootfstype"   , "ext2"])
    call(["fw_setenv", "usb_rootdelay"    , "2"])
    call(["fw_setenv", "usb_load_uimage"  , "mw 0x800000 0 1; ext2load usb $usb_device 0x800000 /boot/uImage"])
    call(["fw_setenv", "usb_boot"         , "if ext2load usb $usb_device 0x1100000 /boot/uInitrd; then bootm 0x800000 0x1100000; else bootm 0x800000; fi;"])
    call(["fw_setenv", "bootcmd_usb"      , "run usb_init; run usb_load_uimage; run set_bootargs_usb; run usb_boot;"])
    call(["fw_setenv", "set_bootargs_usb" , "setenv bootargs console=$console root=$usb_root rootdelay=$usb_rootdelay rootfstype=$usb_rootfstype $mtdparts"])
    call(["fw_setenv", "ubi_part"         , "data"])
    call(["fw_setenv", "ubi_volume"       , "rootfs"])
    call(["fw_setenv", "bootcmd_ubi"      , "ubi part $ubi_part; ubifsmount $ubi_volume; ubifsload 0x800000 /boot/uImage; run set_bootargs_ubi; bootm 0x800000;"])
    call(["fw_setenv", "set_bootargs_ubi" , "setenv bootargs console=$console ubi.mtd=$ubi_part root=ubi0:$ubi_volume rootfstype=ubifs $mtdparts lpj=5955584"])
    call(["fw_setenv", "led_init"         , "green blinking"])
    call(["fw_setenv", "led_exit"         , "green off"])
    call(["fw_setenv", "led_error"        , "orange blinking"])
    call(["fw_setenv", "stdin"            , "serial"])
    call(["fw_setenv", "stdout"           , "serial"])
    call(["fw_setenv", "stderr"           , "serial"])
    call(["fw_setenv", "ethact"           , "egiga0"])
    call(["fw_setenv", "serverip"         , "192.168.0.2"])
    call(["fw_setenv", "ipaddr"           , "192.168.0.42"])
    call(["fw_setenv", "if_netconsole"    , "ping $serverip"])
    call(["fw_setenv", "start_netconsole" , "setenv ncip $serverip; setenv stdin nc; setenv stdout nc; setenv stderr nc; version;"])
    call(["fw_setenv", "read_uImage1"     , "nand read.e 0x800000 0x100000 0x300000"])
    call(["fw_setenv", "read_uImage2"     , "nand read.e 0x800000 0x400000 0x300000"])
    call(["fw_setenv", "bootcmd_nand"     , "run set_bootargs_ubi; run read_uImage1; bootm 0x800000; run read_uImage2; bootm 0x800000"])
    call(["fw_setenv", "bootdelay"        , "1"])
    
if __name__ == "__main__":
    clear()
    setup()

