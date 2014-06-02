#!/bin/sh
sleep 10
cd /var/www/html/cm/
php /var/www/html/cm/bt_daemon.php >> btds/btd_output &
