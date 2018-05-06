#!/bin/bash
kernel=$1
shift
opts=
addopt(){
    opts="$opts $*"
}

#    -kernel /tftpboot/uImage
addopt -M s3cwince
addopt -cpu 1
addopt -m 256
addopt -kernel $kernel
addopt -pflash $kernel
#addopt -mtdblock $kernel
addopt -sd flash-sd.bin
addopt -serial stdio
addopt -show-cursor
addopt -usb
addopt -usbdevice keyboard
addopt -usbdevice mouse
# addopt -net nic,vlan=0
# addopt -net tap,vlan=0,ifname=tap0
addopt -monitor telnet::5555,server,nowait

#export MINI2440_BOOT=nand
#export MINI2440_BOOT=nor
if [ ! -z "$MINI2440_BOOT" ]; then
    addopt -mtdblock flash-$MINI2440_BOOT.bin
fi

#if [ ! -f "$name_nand" ]; then
#    echo $0 : creating NAND empty image : "$name_nand"
#    dd if=/dev/zero of="$name_nand" bs=2112 count=65536
#    echo "** NAND file created - make sure to 'nand scrub' in u-boot"
#fi
#Debug/qemu-system-arm \
#
arm-softmmu/qemu-system-arm \
    $opts \
    $* \
#
