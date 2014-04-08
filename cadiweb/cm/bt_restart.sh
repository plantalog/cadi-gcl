#!/bin/bash


kill -9 $(pidof rfcomm)
rfcomm release 0
 hciconfig hci0 down
 systemctl stop bluetooth
rm -rf /dev/rfcomm0
rm -rf /dev/rfcomm1
rm -rf /dev/rfcomm2
rmmod btusb
modprobe btusb
 systemctl start bluetooth
 hciconfig hci0 up




