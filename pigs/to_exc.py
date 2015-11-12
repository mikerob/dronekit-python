from dronekit import connect
from dronekit.lib import APIException
from serial.serialutil import SerialException
from socket import error as SocketError
import sys
from time import sleep

conn_strs=[]
conn_strs.append("/dev/ttyAMA0")
conn_strs.append("127.0.0.1:15550")
conn_strs.append("/dev/ttyACM0")
conn_strs.append("/dev/ttyUSB0")

print conn_strs

err_list=[]

for c_str in conn_strs:
  try:
    
    print "\n\nConnecting: %s" % c_str
    for a in range(4):
      print a
      sleep(1) 
    print "Go!"
    if 'tty' in c_str:
      vehicle=connect(c_str, baud=57600, heartbeat_timeout=6)
    else:    
      vehicle=connect(c_str, heartbeat_timeout=6)
  except APIException as ai:
    print "APIException fail - Something bad happened"
    print type(ai)
    print ai.args
    print ai.message
    err_list.append((c_str,"timeout"))
  except SocketError as ai:
    print "Socket error"
    print ai.errno
    print ai.strerror
    err_list.append((c_str, "bad ip"))
    # 99 - Cannot assign requested address (bad IP)
    # Bad port times out but doesn't fail
  except SerialException as ai:
    print "Serial port error:"
    print ai
    print "No:",ai.errno
    print "Filename:",ai.filename
    print ai.args
    print ai.message
    if 'could not open port' in ai.message:
      print "Bad port, doofus"
      err_list.append((c_str, "bad serial port"))
    else:
      print "Serial error"
      err_list.append((c_str, "serial error"))
  except:
    print "Unexpected", sys.exc_info()[0]
    err_str = sys.exc_info()[0] 
  else:
    print "It worked"
    good_conn=c_str
    err_list.append((c_str, "connect"))
    break

print "Now I'm connected to ",good_conn
print err_list
sleep(10)
vehicle.close()

