#!/bin/sh

tmproot=/tmp/bhroot
archive=$1

ls="${tmproot}/busybox ls"
rm="${tmproot}/busybox rm"
cp="${tmproot}/busybox cp"
tar="${tmproot}/busybox tar"


create_tmproot()
{
    echo Copying necessary files to ${tmproot}
    mkdir -p ${tmproot}
    cp -f /lib/ld-uClibc.so.0 ${tmproot}
    cp -f /lib/libc.so.0 ${tmproot}
    cp -f /lib/libgcc_s.so ${tmproot}
    cp -f /bin/busybox ${tmproot}
    cp -f /usr/sbin/nanddump ${tmproot}
    cp -f /etc/network/interfaces ${tmproot}
    export LD_LIBRARY_PATH=${tmproot}
}


erase_root()
{
    for item in `${ls} /`
    do
        if [ "${item}" != "dev" -a "${item}" != "proc" -a "${item}" != "root" -a "${item}" != "tmp" -a "${item}" != "lost+found" -a "${item}" != "sys" ]
        then
            echo Removing ${item}
            ${rm} -rf /${item}
        fi
    done
}


install_root()
{
    echo Installing new system
    echo ${tar} -C / xvf ${archive}
    echo Restoring /etc/network/interfaces
    echo ${cp} -f ${tmproot}/interfaces /etc/network/interfaces
}

create_tmproot
erase_root
install_root

