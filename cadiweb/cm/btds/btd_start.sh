#!/bin/sh
sleep 10
cd /var/www/cm/
php /var/www/cm/bt_daemon.php >> btds/btd_output &
