CBD: Cadi Bluetooth Daemon
=============

Cadi integrated Bluetooth module communicates using serial protocol wirelessly.
This is used, when Cadi hadrware is controlled with server.
Cadiweb is used as controlling software, that is running on server and client side.
Server side presents user web-interface and exposed to end user as proxy for Cadi data
Client side equipped with web-browser establishes connection with Cadiweb server and starts processing the Cadi data got from Cadiweb server.
Cadiweb server has a communication daemon, that manages the Cadi Bluetooth Connection and data flows.
It produces the number of files, during it's work:
- cadi_status.csv for keeping comma separated values of status data, got from Cadi controller.
- serialresp.out - raw serial output from Cadi
- btds/btd_output for storing CBD log
- daemon_cmd - keeps data sent to daemon from Cadiweb. This file used to control the CBDaemon
- btds/btd.conf - config file for daemon.

The installer script adds entry into OS startup scripts for CBD proper start alongside with OS (for example, in Ubuntu, CBD startup scpit execution call added into /etc/rc.local).

CBDaemon also responsible for making photoshots using server webcam hardware.
The frequency of photoshots is adjusted within btd.conf file.

CBD architecture
============
Taking for instance Ubuntu Linux daemon written on PHP.
There is a main loop, that has a delay at the end of iteration. Tis delay added to hold daemon for a while. This keeps daemon running more smoothly, not making 100% CPU usage, with more predictable response timings.
This delay is called Command Scan Delay, This is because, each loop iteration, daemon reads a new command from the daemon_cmd file to execute. Default delay is 25000 microseconds (25ms).
This means 40 "scans" per second, if the code beside this 25ms delay runs in 0ms.
Having 9600baud exchange rate (~1KB/s) and 40bytes status packet size we could get up to 25 packets per second (in perfect conditions).
So, for 40fps, having 25 packets in second seems to not to be a problem.
Anyway, beside this there are another variables, that adjust the data exchange
Response Parser Divider shows how often daemon should attempt to parse the serialresp.out file for new Cadi packet. Let's say, the default fps is 25+ ms, and Response Parser Divider is set to 5. This will mean, that packet parsing attempts will be performed every 5th loop's cycle - every 125+ ms
File Size Difference - is amount of bytes the serialresp.out should be changed before the parsing attempt to be made.
Status Packet Ping Divider (SPPD) - is divider of main loop, indicating how often (each Nth run of main loop) to send status ping packet to Cadi.
Serial Response Tail Read Size (SRTRS) - this number shows amount of bytes to be read from the tail of serialresp.out file to parse the Cadi output.


