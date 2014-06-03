#!/bin/bash
# this script "installs" cadiweb on Ubuntu 12.04 LTS system
# http://embeddedprogrammer.blogspot.com.es/2012/06/ubuntu-openingusing-serial-ports-as.html
# http://embeddedprogrammer.blogspot.com.es/2012/06/ubuntu-hacking-hc-06-bluetooth-module.html



export DEBIAN_FRONTEND=noninteractive

echo mysql-server-5.5 mysql-server/root_password password xyzzy | debconf-set-selections
echo mysql-server-5.5 mysql-server/root_password_again password xyzzy | debconf-set-selections

echo 'installing LAMP server for Ubuntu'
sudo apt-get -q -y install lamp-server^
echo 'installing fswebcam application to take photo shots'
sudo apt-get -q -y install fswebcam
echo 'creating directory /var/www/cm'
sudo mkdir /var/www/cm
echo 'creating directory /var/www/cm/btds for Bluetooth Daemon data and settings'
sudo mkdir /var/www/cm/btds/
echo 'changing ownership to www-data for directory /var/www'
chown www-data:www-data /var/www
echo 'changing ownership to www-data for directory /var/www/cm'
chown www-data:www-data /var/www/cm

echo 'changing permissions for directory /var/www/cm/btds'
sudo chmod 777 /var/www/cm/btds

echo 'changing ownership for directory /var/www/cm/btds'
sudo chown www-data:www-data /var/www/cm/btds

echo 'creating/flushing /var/www/cm/daemon_cmd file (Cadi BT Daemon command log)'
sudo echo > /var/www/cm/daemon_cmd

echo 'changing permissions for file daemon_cmd'
sudo chmod 777 /var/www/cm/daemon_cmd

echo 'changing ownership to www-data for /var/www/cm/daemon_cmd'
sudo chown www-data:www-data /var/www/cm/daemon_cmd

echo 'creating/flushing /var/www/cm/btds/btd_output file (Cadi BTD output log)'
sudo echo > /var/www/cm/btds/btd_output

echo 'changing permissions for /var/www/cm/btds/btd_output'
sudo chmod 777 /var/www/cm/btds/btd_output

echo 'creating/flushing Cadi status CSV file (/var/www/cm/cadi_status.csv)'
sudo echo > /var/www/cm/cadi_status.csv

echo 'changing Cadi status CSV file ownership'
sudo chown www-data:www-data /var/www/cm/cadi_status.csv

echo 'changing Cadi status CSV file permissions'
sudo chmod 755 /var/www/cm/cadi_status.csv

echo 'creating/flushing Cadi serial data dump log file (/var/www/cm/serialresp.out)'
sudo echo > /var/www/cm/serialresp.out

echo 'changing ownership for Cadi serial data dump log file'
sudo chown www-data:www-data /var/www/cm/serialresp.out

echo 'changing permissions for Cadi serial data dump log file'
sudo chmod 755 /var/www/cm/serialresp.out

echo 'changing permissions for Cadiweb parent directory (/var/www/html)'
sudo chmod 777 /var/www

echo 'changing ownership for Cadi parent directory'
sudo chown www-data:www-data /var/www

echo 'changing permissions for all files inside Cadiweb modules files (/var/www/cm)'
cd /var/www/cm
sudo chmod 755 *

echo 'removing default Apache index.html page'
sudo rm -rf /var/www/index.html


# add BT daemon startup line into /etc/rc.local
sudo grep -Fxq "/bin/sh /var/www/cm/btds/btd_start.sh" /etc/rc.local
RESPONSE=$?
if [ $RESPONSE -eq 1 ];then
	sudo sed -i -e '$i/bin/sh /var/www/cm/btds/btd_start.sh' /etc/rc.local ;
	echo 'startup string added to /etc/rc.local';
else
	echo 'startup string already exists in /etc/rc.local' ;
fi

# getting Cadiweb repository files (http://stackoverflow.com/questions/600079/is-there-any-way-to-clone-a-git-repositorys-sub-directory-only)
sudo apt-get -q -y install git
sudo mkdir /tmp/cadi-gcl
sudo chmod 777 /tmp/cadi-gcl
cd /tmp/
sudo git init cadi-gcl
cd cadi-gcl
sudo git remote add -f origin https://github.com/plantalog/cadi-gcl.git
sudo git config core.sparsecheckout true
sudo echo "cadiweb/" >> .git/info/sparse-checkout
sudo git pull origin master

cd /tmp/cadi-gcl/cadiweb/
sudo cp -rf * /var/www/

echo 'changing permissions for Cadi BTDaemon startup script (/var/www/cm/btds/btd_start.sh)'
sudo chmod 777 /var/www/cm/btds/btd_start.sh
# sudo chown www-data:www-data /var/www/cm/btds/btd_start.sh
# sudo l2ping 20:13:06:14:34:06
# http://wiki.openmoko.org/wiki/Manually_using_Bluetooth
# http://bluelimn.tistory.com/m/post/715

echo 'changing permissions for Cadi BTDaemon (/var/www/cm/bt_daemon.php)'
sudo chmod 777 /var/www/cm/bt_daemon.php

# http://scottada.ms/article/adding-so-called-virtual-bluetooth-serial-port-ubuntu-1110-using-atheros-ar3011-based-usb
sdptool add --channel=14 SP
cd /var/lib/bluetooth/
echo 'listing available adapters (hciconfig -a)'
hciconfig -a
HCIMAC=$(hciconfig | grep -o -E '([[:xdigit:]]{1,2}:){5}[[:xdigit:]]{1,2}')
cd $HCIMAC
echo -e 'adding PIN codes to /var/lib/bluetooth/'$HCIMAC
sudo echo -e '20:13:06:19:15:11 1234\n20:13:06:14:34:06 1234\n' | sudo tee pincodes

echo 'creating default BTDaemon config file'
sudo echo '80,25000,37,5,0,150,3' >> /var/www/cm/btds/btd.conf

sudo chmod 777 /var/www/cm/btds/btd.conf
# hcitool -i hci0 info <bdaddr>

echo '### IT IS RECOMMENDED TO RESTART COMPUTER TO APPLY CHANGES ###'

exit 0
