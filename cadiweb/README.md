Cadiweb
=======

This folder contains Cadiweb server software needed to control the Cadis remotely.

INSTALLATION
=============

For ArchLinux on Raspberry Pi run in your Pi's terminal session:
	wget https://github.com/plantalog/cadi-gcl/raw/master/cadiweb/install-arclinux0614_pi.sh -O /tmp/install.sh
	chmod 777 /tmp/install.sh
	/bin/sh /tmp/install.sh

For Ubuntu 12.04 LTS run the following lines in your Ubuntu terminal:
	wget https://github.com/plantalog/cadi-gcl/raw/master/cadiweb/install-ubuntu1204.sh -O /tmp/install.sh
	chmod 777 /tmp/install.sh
	sudo /tmp/install.sh

For Ubuntu 14.04 LTS run the following lines in your Ubuntu terminal:
	wget https://github.com/plantalog/cadi-gcl/raw/master/cadiweb/install-ubuntu1404.sh -O /tmp/install.sh
	chmod 777 /tmp/install.sh
	sudo /tmp/install.sh

This scripts silently install LAMP server, git, fswebcam application, adds the Cadi Bluetooth Daemon startup line into /etc/rc.local and copies the Github repository folder with cadiweb into Apache's default document root directory.



CADIWEB BLUETOOTH DAEMON
==========================
This daemon needs to be running for communication between Cadiweb PHP/JS application and Cadi controller.
It uses 5 files in it's work:
- daemon_cmd
	When some action is made in Cadiweb panel, it is written as a command into this file. CBDaemon reads this file and runs the command then.

- serialresp.out
	This is raw output from bluetooth serial port, Cadi controller attached to

- cadi_status.csv
	This file generated by CBDaemon based on status response from Cadi

- btd_output
	During it's work, daemon generate log file, that is stored here.

- btd.conf
	Config file, containing the CBDaemon settings

Cadi Bluetooth Daemon startup with systemd in ArchLinux
	While installation of Cadiweb on Raspberry Pi's ArchLinux OS, the CBTDaemon systemd service is enabled to start up with ArchLinux. Its name is cbtd.service and it could be manually run with systemctl
	systemctl enable cbtd.service
and to run it manually execute:
	systemctl start cbtd.service

