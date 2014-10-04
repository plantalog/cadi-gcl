#!/bin/sh
sleep 10
cd /srv/http/cm/
php /srv/http/cm/bt_daemon.php >> btds/btd_output &
systemctl start httpd.service
systemctl start bluetooth
hciconfig hci0 up



