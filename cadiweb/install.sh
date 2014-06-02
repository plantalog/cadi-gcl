#!/bin/bash
# this script "installs" cadiweb on Ubuntu 12.04 LTS system

export DEBIAN_FRONTEND=noninteractive

echo mysql-server-5.5 mysql-server/root_password password xyzzy | debconf-set-selections
echo mysql-server-5.5 mysql-server/root_password_again password xyzzy | debconf-set-selections

echo 'installing LAMP server for Ubuntu'
sudo apt-get -q -y install lamp-server^
echo 'installing fswebcam application to take photo shots'
sudo apt-get -q -y install fswebcam
echo 'creating directory /var/www/html/cm'
sudo mkdir /var/www/html/cm
echo 'creating directory /var/www/html/cm/btds for Bluetooth Daemon data and settings'
sudo mkdir /var/www/html/cm/btds/
echo 'changing ownership to www-data for directory /var/www/html'
chown www-data:www-data /var/www/html
echo 'changing ownership to www-data for directory /var/www/html/cm'
chown www-data:www-data /var/www/html/cm

echo 'changing permissions for directory /var/www/html/cm/btds'
sudo chmod 777 /var/www/html/cm/btds

echo 'changing ownership for directory /var/www/html/cm/btds'
sudo chown www-data:www-data /var/www/html/cm/btds

echo 'creating/flushing /var/www/html/cm/daemon_cmd file (Cadi BT Daemon command log)'
sudo echo > /var/www/html/cm/daemon_cmd

echo 'changing permissions for file daemon_cmd'
sudo chmod 777 /var/www/html/cm/daemon_cmd

echo 'changing ownership to www-data for /var/www/html/cm/daemon_cmd'
sudo chown www-data:www-data /var/www/html/cm/daemon_cmd

echo 'creating/flushing /var/www/html/cm/btds/btd_output file (Cadi BTD output log)'
sudo echo > /var/www/html/cm/btds/btd_output

echo 'changing permissions for /var/www/html/cm/btds/btd_output'
sudo chmod 777 /var/www/html/cm/btds/btd_output

echo 'creating/flushing Cadi status CSV file (/var/www/html/cm/cadi_status.csv)'
sudo echo > /var/www/html/cm/cadi_status.csv

echo 'changing Cadi status CSV file ownership'
sudo chown www-data:www-data /var/www/html/cm/cadi_status.csv

echo 'changing Cadi status CSV file permissions'
sudo chmod 755 /var/www/html/cm/cadi_status.csv

echo 'creating/flushing Cadi serial data dump log file (/var/www/html/cm/serialresp.out)'
sudo echo > /var/www/html/cm/serialresp.out

echo 'changing ownership for Cadi serial data dump log file'
sudo chown www-data:www-data /var/www/html/cm/serialresp.out

echo 'changing permissions for Cadi serial data dump log file'
sudo chmod 755 /var/www/html/cm/serialresp.out

echo 'changing permissions for Cadiweb parent directory (/var/www/html)'
sudo chmod 777 /var/www/html

echo 'changing ownership for Cadi parent directory'
sudo chown www-data:www-data /var/www/html

echo 'changing permissions for all files inside Cadiweb modules files (/var/www/html/cm)'
cd /var/www/html/cm
sudo chmod 755 *

echo 'removing default Apache index.html page'
sudo rm -rf /var/www/html/index.html


# add BT daemon startup line into /etc/rc.local
sudo grep -Fxq "/bin/sh /var/www/html/cm/btds/btd_start.sh" /etc/rc.local
RESPONSE=$?
if [ $RESPONSE -eq 1 ];then
	sudo sed -i -e '$i/bin/sh /var/www/html/cm/btds/btd_start.sh' /etc/rc.local ;
	echo 'startup string added to /etc/rc.local';
elif [ $RESPONSE -eq 0 ];then
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
sudo cp -rf * /var/www/html/

echo 'changing permissions for Cadi BTDaemon startup script (/var/www/html/cm/btds/btd_start.sh)'
sudo chmod 777 /var/www/html/cm/btds/btd_start.sh

echo '### IT IS RECOMMENDED TO RESTART COMPUTER TO APPLY CHANGES ###'

exit 0
