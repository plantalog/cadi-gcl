import bluetooth
import serial, sys,os
name1=sys.argv[1]
f = open('serialresp.out','w')

bt_addr = name1
port = 1

sock=bluetooth.BluetoothSocket( bluetooth.RFCOMM )
sock.connect((bt_addr, port))


l=sock.recv(500)
curinsize = os.stat('cadi_input').st_size
i=0
while True:
#swhile (len(l)<500):
  statinfo = os.stat('cadi_input')
  if (statinfo.st_size>curinsize):
    curinsize=statinfo.st_size
    finput = open('cadi_input','r')
    finput.seek(-2, os.SEEK_END)
    command = finput.read(1);
    sock.send(command)
  i+=1
  l=sock.recv(10)
  f.write(l)
  f.flush()
  
sock.close()
