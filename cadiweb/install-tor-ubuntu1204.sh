#!/bin/bash
# this script "installs" Tor for Cadiweb on Ubuntu 12.04 LTS system

# sudo deb http://deb.torproject.org/torproject.org precise main
sudo apt-add-repository 'deb http://deb.torproject.org/torproject.org precise main'
sudo gpg --keyserver keys.gnupg.net --recv 886DDD89
sudo gpg --export A3C4F0F979CAA22CDBA8F512EE8CBC9E886DDD89 | sudo apt-key add -
sudo apt-get update
sudo dpkg -r tor-browser tor-geoipdb
sudo apt-get install -f
sudo apt-get -q -y install tor

sudo mkdir /var/www/tor
sudo mkdir /var/www/tor/hs
sudo chown debian-tor:debian-tor /var/www/tor/hs
sudo chmod 777 /var/www/tor/hs


# check if Hidden Service config line already exists
sudo grep -Fxq "HiddenServicePort 80 127.0.0.1:80" /etc/tor/torrc
RESPONSE=$?
if [ $RESPONSE -eq 1 ];then
#	sudo sed -i -e '$i/bin/sh /var/www/cm/btds/btd_start.sh' /etc/rc.local ;
	sudo awk '$0 ~ str{print NR-1 FS b}{b=$0}' str="This section is just for location-hidden services" /etc/tor/torrc
	LINENR=$?
	# use LINENR+9 to get desired line add position
	sudo sed -i -e '70i\HiddenServiceDir /var/www/tor/hs' /etc/tor/torrc
	sudo sed -i -e '70i\HiddenServicePort 80 127.0.0.1:80' /etc/tor/torrc
	echo 'Tor hidden service config lines added to /etc/tor/torrc';
else
	echo 'Tor Hidden service config string already exists in /etc/tor/torrc' ;
fi

sudo service tor restart


exit 0
