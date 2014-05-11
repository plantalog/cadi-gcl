#!/bin/bash
# this script "installs" cadiweb on Ubuntu 12.04 LTS system
sudo apt-get install lamp-server^
sudo apt-get install fswebcam
echo /var/www/cm/daemon_cmd
chmod 777 /var/www/cm/daemon_cmd
echo /var/www/cm/btds/btd_output
chmod 777 /var/www/cm/btds/btd_output

echo /var/www/cm/cadi_status.csv
chown www-data:www-data /var/www/cm/cadi_status.csv
chmod 755 /var/www/cm/cadi_status.csv

echo /var/www/cm/serialresp.out
chown www-data:www-data /var/www/cm/serialresp.out
chmod 755 /var/www/cm/serialresp.out

cd /var/www/cm
chmod 755 *



