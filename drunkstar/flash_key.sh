DEV=/dev/sdb
MOUNTPT=/tmp/bhusbkey
IMAGES=/home/thier/Informatique/binhouseteam/bhware-open/drunkstar/buildroot-2012.08/output/images

# format key
umount ${DEV}1
umount ${DEV}2
mkfs.ext2 -L kernel ${DEV}1
mkfs.ext4 -L rootfs ${DEV}2

# make mount point
mkdir -p ${MOUNTPT}

# copy kernel
mount ${DEV}1 ${MOUNTPT}
mkdir ${MOUNTPT}/boot
cp ${IMAGES}/uImage ${MOUNTPT}/boot
umount ${DEV}1

# copy rootfs
mount ${DEV}2 ${MOUNTPT}
tar -x -j -C ${MOUNTPT} -f ${IMAGES}/rootfs.tar.bz2
cp ${IMAGES}/uImage ${MOUNTPT}/root
cp ${IMAGES}/rootfs.ubi ${MOUNTPT}/root
umount ${DEV}2

rm -rf ${MOUNTPT}
