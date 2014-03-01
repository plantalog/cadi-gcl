#!/bin/bash

# $1 - mac
# $2 - rfcomm

#restart bluetooth service
hciconfig hci0 down
systemctl stop bluetooth
systemctl start bluetooth
hciconfig hci0 up

# create rfcomm link
rfcomm connect /dev/$2 $1
stty -F /dev/$2 raw
stty -F /dev/$2 -echo -echoe -echok

# write to port using echo 'hello world' > /dev/$2

