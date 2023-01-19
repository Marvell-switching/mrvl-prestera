#!/usr/bin/env bash
#
# A script to build mvDrvDrv.ko manually
#
# Edit next line and set $DIST to kernel's source path
DIST=/usr/src/linux-headers-`uname -r`

#
# These variables are for Marvell's ARM LSP only:
#
#export ARCH=arm
#export CROSS_COMPILE=/swtools/devtools/gnueabi/arm_le/arm-none-linux-gnueabi-versions/armv7-marvell-linux-gnueabi-softfp_i686/bin/arm-marvell-linux-gnueabi-



make -C $DIST M=`pwd` modules
