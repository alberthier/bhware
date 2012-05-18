#!/usr/bin/python


import sys
import os
from subprocess import *


if __name__ == "__main__":

    if len(sys.argv) < 2 or not os.path.exists(sys.argv[1]) or not sys.argv[1].startswith("/dev/sd") or sys.argv[1].startswith("/dev/sda"):
        print("You need to provide a device name")
        sys.exit(-1)

    device_name = sys.argv[1]

    bhware_drunkstar_root = os.path.abspath(os.path.dirname(sys.argv[0]))
    bhware_root = os.path.dirname(bhware_drunkstar_root)

    for dev in os.listdir("/dev"):
        pdev = "/dev/" + dev
        if pdev.startswith(device_name) and pdev != device_name:
            call("umount {}".format(pdev), shell = True)

    call("mkfs.ext3 {}1".format(device_name), shell = True)
    call("mount {}1 /mnt".format(device_name), shell = True)
    Popen("tar xf {}".format(bhware_drunkstar_root + "/buildroot-2012.02/output/images/rootfs.tar"), shell = True, cwd = "/mnt").wait()
    call("mount {}1 /mnt".format(device_name), shell = True)
    call('sed -i -e "s/<ssid>/BHTeam/g" /mnt/etc/wpa_supplicant/wpa_supplicant.conf', shell = True)
    call('sed -i -e "s/<personal-secret-key>/BHTeamKey/g" /mnt/etc/wpa_supplicant/wpa_supplicant.conf', shell = True)
    Popen("hg clone {} bhware".format(bhware_root), shell = True, cwd = "/mnt/root").wait()
    call("umount /mnt", shell = True)
