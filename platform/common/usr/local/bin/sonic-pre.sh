#!/bin/bash

MACHINE=$(uname -m)

if [ -f /usr/local/bin/sonic_firstboot ]; then
    rm -f /usr/local/bin/sonic_firstboot
    case $MACHINE in
        armv7l)
            DELAY=30
            ;;
        aarch64|x86_64)
            DELAY=2
            ;;
        *)
            DELAY=0
            ;;
    esac
else
    case $MACHINE in
        armv7l)
            DELAY=20
            ;;
        aarch64|x86_64)
            DELAY=0
            ;;
        *)
            DELAY=0
            ;;
    esac
fi

echo "sonic-pre start with delay $DELAY sec" > /dev/kmsg
if [ "$DELAY" -eq 0 ]; then exit 0; fi
sleep $DELAY
echo "sonic-pre end. Start sonic" > /dev/kmsg

