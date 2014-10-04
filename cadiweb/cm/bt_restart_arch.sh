#!/bin/bash


kill -9 $(pidof rfcomm)
RFTMP=$(rfcomm)
RFNUM=${RFTMP:6:1}
rfcomm release ${RFNUM}
hciconfig hci0 down
systemctl stop bluetooth
rm -rf /dev/rfcomm${RFNUM}
rmmod btusb
modprobe btusb
systemctl start bluetooth
hciconfig hci0 up

echo 'bt_restart Archlinux edition executed'




