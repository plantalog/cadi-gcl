#!/bin/bash

echo "deb     http://deb.torproject.org/torproject.org precise main" >> /etc/apt/sources.list
gpg --keyserver keys.gnupg.net --recv 886DDD89
gpg --export A3C4F0F979CAA22CDBA8F512EE8CBC9E886DDD89 | sudo apt-key add -
sudo apt-get update
sudo apt-get install lamp-server^
cd /var/www
sudo echo > /var/www/cm/daemon_cmd
sudo chown www-data:www-data daemon_cmd
echo
sudo apt-get install tor
sudo mkdir /var/www/tor
sudo mkdir /var/www/tor/cw/cm
cd /var/www/tor/cw
sudo echo > daemon_cmd
sudo chown www-data:www-data daemon_cmd
sudo chown debian-tor:debian-tor /var/www/tor/cw/



