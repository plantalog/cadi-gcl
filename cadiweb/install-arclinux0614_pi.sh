#!/bin/bash
# this script installs Cadiweb on Archlinux for Raspberry Pi

echo 'Refreshing pacman databases'
pacman -Syu --noconfirm

echo 'Installing RPi watchdog (http://blog.ricardoarturocabral.com/2013/01/auto-reboot-hung-raspberry-pi-using-on.html)'
echo "bcm2708_wdog" | tee /etc/modules-load.d/bcm2708_wdog.conf
pacman -S --noconfirm watchdog
systemctl enable watchdog
echo 'Uncomment the line that starts with #watchdog-device'
echo 'Uncomment the line that says #max-load-1 = 24'
nano /etc/watchdog.conf
systemctl start watchdog.service
echo 'END OF Watchdog install'

echo 'Install LAMP server'
pacman -S --noconfirm apache php php-apache mariadb
echo 'adding Apache daemon into startup'
systemctl enable httpd

echo 'Configure Apache and PHP to work together (https://wiki.archlinux.org/index.php/LAMP)'


echo '=== configuring bluetooth'

cd /var/lib/bluetooth/
echo 'listing available adapters (hciconfig -a)'
hciconfig -a
HCIMAC=$(hciconfig | grep -o -E '([[:xdigit:]]{1,2}:){5}[[:xdigit:]]{1,2}')
cd $HCIMAC
echo -e 'adding PIN codes to /var/lib/bluetooth/'$HCIMAC
CADIMAC=$(hcitool scan | grep -o -E '([[:xdigit:]]{1,2}:){5}[[:xdigit:]]{1,2}')
echo 'adding '
echo -e $CADIMAC' 1234' | tee pincodes
echo 'pincode 1234 added for device '$CADIMAC

echo 'Adding date timezone to php.ini: Europe/Madrid'



# getting Cadiweb repository files (http://stackoverflow.com/questions/600079/is-there-any-way-to-clone-a-git-repositorys-sub-directory-only)
pacman -S --noconfirm git
mkdir /tmp/cadi-gcl
chmod 777 /tmp/cadi-gcl
cd /tmp/
git init cadi-gcl
cd cadi-gcl
git remote add -f origin https://github.com/plantalog/cadi-gcl.git
git config core.sparsecheckout true
echo "cadiweb/" >> .git/info/sparse-checkout
git pull origin master

cd /tmp/cadi-gcl/cadiweb/
cp -rf * /srv/http/

# chown http:http /srv/http/cm/btds/btd_start.sh
# l2ping 20:13:06:14:34:06
# http://wiki.openmoko.org/wiki/Manually_using_Bluetooth
# http://bluelimn.tistory.com/m/post/715

echo 'changing permissions for Cadi BTDaemon (/srv/http/cm/bt_daemon.php)'
chmod 777 /srv/http/cm/bt_daemon.php





echo 'create directory /srv/http/cm (ArchLinux Raspberry typical)'
mkdir /srv/http/cm
echo 'creating directory /srv/http/cm/btds for Bluetooth Daemon data and settings'
mkdir /srv/http/cm/btds/
echo 'changing ownership to http for directory /srv/http'
chown http:http /var/www
echo 'changing ownership to http for directory /srv/http/cm'
chown http:http /srv/http/cm


echo 'flushes file before downloading new settings from Cadi'
echo > /srv/http/cm/cadi_settings_dump
echo 'setting permissions for Cadi settings dump file'
chmod 777 /srv/http/cm/cadi_settings_dump
echo 'setting permissions for Cadi settings csv file'
chmod 777 /srv/http/cm/cadi_settings_conf.csv


# add BT daemon startup script call line into /etc/rc.local
echo 'create /etc/rc.local if not exist'
echo >> /etc/rc.local
grep -Fxq "/bin/sh /srv/http/cm/btds/btd_start.sh" /etc/rc.local
RESPONSE=$?
if [ $RESPONSE -eq 1 ];then
	sed -i -e '$i/bin/sh /srv/http/cm/btds/btd_start.sh\n /bin/sh /srv/http/cm/bt_restart_arch.sh' /etc/rc.local ;
	echo 'startup string added to /etc/rc.local';
else
	echo 'startup string already exists in /etc/rc.local' ;
fi


# Cadi BTDaemon startup script and systemd service
echo 'generating Cadi BTDaemon startup script'
echo -e '#!/bin/sh\nsleep 10\ncd /srv/http/cm/\nphp /srv/http/cm/bt_daemon.php >> btds/btd_output &\nhciconfig hci0 up' > /usr/lib/systemd/scripts/btd_start.sh

echo 'creating systemd service'
echo '[Unit]\n
Description=Cadi BTDaemon\n
\n
[Service]\n
Type=oneshot\n
ExecStart=/usr/lib/systemd/scripts/btd_start.sh\n
RemainAfterExit=yes\n
\n
[Install]\n
WantedBy=multi-user.target\n' > /usr/lib/systemd/system/cbtd.service

echo 'enabling Cadi BTDaemon service @boot startup'
systemctl enable cbtd.service



echo 'changing permissions for Cadi BTDaemon startup script (/srv/http/html/cm/btds/btd_start.sh)'
chmod 777 /srv/http/cm/btds/btd_start.sh







echo 'backing up cadi_settings file';
cp -rf /srv/http/cm/cadi_settings /tmp

echo 'changing permissions for directory /srv/http/cm/btds'
chmod 777 /srv/http/cm/btds

echo 'changing ownership for directory /srv/http/cm/btds'
chown http:http /srv/http/cm/btds

echo 'creating/flushing /srv/http/cm/daemon_cmd file (Cadi BT Daemon command log)'
echo > /srv/http/cm/daemon_cmd

echo 'changing ownership to http for /srv/http/cm/daemon_cmd'
chown http:http /srv/http/cm/daemon_cmd

echo 'changing permissions for file daemon_cmd'
chmod 777 /srv/http/cm/daemon_cmd

echo 'creating/flushing /srv/http/cm/btds/btd_output file (Cadi BTD output log)'
echo > /srv/http/cm/btds/btd_output

echo 'changing permissions for /srv/http/cm/btds/btd_output'
chmod 777 /srv/http/cm/btds/btd_output

echo 'creating/flushing Cadi status CSV file (/srv/http/cm/cadi_status.csv)'
echo > /srv/http/cm/cadi_status.csv

echo 'changing Cadi status CSV file ownership'
chown http:http /srv/http/cm/cadi_status.csv

echo 'changing Cadi status CSV file permissions'
chmod 777 /srv/http/cm/cadi_status.csv
chmod 777 /srv/http/cm/status_view_1.php
chmod 777 /srv/http/cm/status_view_2.php

echo 'creating/flushing Cadi serial data dump log file (/srv/http/cm/serialresp.out)'
echo > /srv/http/cm/serialresp.out

echo 'changing ownership for Cadi serial data dump log file'
chown http:http /srv/http/cm/serialresp.out

echo 'changing permissions for Cadi serial data dump log file'
chmod 755 /srv/http/cm/serialresp.out

echo 'changing ownership for Cadi parent directory'
chown http:http /var/www

echo 'changing permissions for Cadiweb parent directory (/srv/http/html)'
chmod 777 /var/www

echo 'changing permissions for all files inside Cadiweb modules files (/srv/http/cm)'
cd /srv/http/cm
chmod 755 *

echo 'removing default Apache index.html page'
rm -rf /srv/http/index.html



echo 'creating default BTDaemon config file'
echo '80,25000,37,5,0,150,3' > /srv/http/cm/btds/btd.conf
chmod 777 /srv/http/cm/btds/btd.conf
# hcitool -i hci0 info <bdaddr>

echo 'creating default SVG config file'
echo '12,100,728,193,9,50,586,393' > /srv/http/cm/svg.conf
chmod 777 /srv/http/cm/svg.conf

echo 'recovering backed up settings file'
cp -rf /tmp/cadi_settings /srv/http/cm/
chown http:http /srv/http/cm cadi_settings

echo '### IT IS RECOMMENDED TO RESTART COMPUTER TO APPLY CHANGES ###'

exit 0
