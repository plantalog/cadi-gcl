#!/bin/sh

# this script starts 3G Internet for Cadi
cd /var/www/cm/btds
sleep 10
nmcli con up id "Tele2 connection"
