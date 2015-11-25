# requires RPi_I2C_driver.py
import RPi_I2C_driver
import time,sys
from commands import getstatusoutput
from os import listdir

from dronekit import connect
from dronekit.lib import APIException
from serial.serialutil import SerialException
from socket import error as SocketError

mylcd = RPi_I2C_driver.lcd()
mylcd.lcd_clear()

connections_cmd = "ifconfig | grep Link"
my_eth_ip = "ip addr show eth0 | grep inet | awk '{print $2}' | cut -d/ -f1"
my_wlan_ip = "ip addr show wlan0 | grep inet | awk '{print $2}' | cut -d/ -f1"

def run_cmd(cmd): 
  #p = Popen(cmd, shell = True, stdout = PIPE) 
  #output = p.communicate()[0] 
  (status, output) = getstatusoutput(cmd)
  if status:
    sys.stderr.write(output)
  return output

def find_conns():
  conns=[]
  dirty_conns=run_cmd(connections_cmd)
  dirty_conns=dirty_conns.split("\n")
  for dirty_conn in dirty_conns:
    conn_name=dirty_conn.split()[0]
    find_ip_str = "ip addr show "+conn_name+" | grep inet | awk '{print $2}' | cut -d/ -f1"
    conn_ip=run_cmd(find_ip_str)
    conns.append((conn_name,conn_ip))
  return conns
  
# Search for TTYs that work for APM 
def find_ttys():
  ttys=[]
  tty_list=listdir('/dev/')
  for tty in tty_list:
    if 'USB' in tty:
      ttys.append(tty)
    if 'AMA' in tty:
      ttys.append(tty)
    if 'ACM' in tty:
      ttys.append(tty)
  return ttys

def clear_line(lcd, line):
  lcd.lcd_display_string("                    ",line)

# Try to connect to a given connection
# Returns (connection_string, status)
def try_connect(lcd, c_str, baud_in=57600, heart_time=15):
  global vehicle
  err_tuple=None
  try:
    print "\n\nConnecting: %s" % c_str
    clear_line(lcd,3)
    clear_line(lcd,4)
    lcd.lcd_display_string("Try:%s" % c_str,3)
    if 'tty' in c_str:
      vehicle=connect(c_str, baud=baud_in, heartbeat_timeout=heart_time)
    else:    
      vehicle=connect(c_str, heartbeat_timeout=heart_time)
  except APIException as ai:
    print "APIException fail - Something bad happened"
    print ai.message
    lcd.lcd_display_string("Timeout",4)
    err_tuple=(c_str,"timeout")
  except SocketError as ai:
    print "Socket error"
    print ai.errno
    lcd.lcd_display_str(ai.strerror,4)
    err_tuple=(c_str, "bad ip")
  except SerialException as ai:
    print "Serial port error:"
    if 'could not open port' in ai.message:
      print ai.message,
      lcd.lcd_display_string("Bad port",4)
      err_tuple=(c_str, "bad serial port")
    else:
      print ai.message
      lcd.lcd_display_string("Serial error",4)
      err_tuple=(c_str, "serial error")
  except:
    print "Unexpected", sys.exc_info()[0]
    lcd.lcd_display(sys.exc_info()[0],4)
    err_tuple=(c_str, sys.exc_info()[0]) 
  else:
    print "It worked"
    good_conn=c_str
    lcd.lcd_display_string("Connected",4)
    err_tuple=(c_str, "connect")
  return err_tuple

conns_list=find_conns()
ttys_list=find_ttys()

print conns_list
print ttys_list

mylcd.lcd_clear()

conn_i=0
tty_i=0
all_conn=False
all_tty=False

min_try_port=14550
max_try_port=14560
try_port=14550
try_baud=57600
ip_fail=True

tty_fail=True
connected = False

while True: 
  # Alternate display fo each IP and serial connection
  conns_list=find_conns()
  ttys_list=find_ttys()

  if conn_i<(len(conns_list)-1):
    conn_i+=1
  else:
    all_conn=True
    conn_i=0

  if tty_i<(len(ttys_list)-1):
    tty_i+=1
  else:
    all_tty=True
    tty_i=0
  
  clear_line(mylcd,1)
  mylcd.lcd_display_string('%s:%s' %(conns_list[conn_i][0], conns_list[conn_i][1]), 1)
  clear_line(mylcd,2)
  mylcd.lcd_display_string('%s:/dev/%s' %(tty_i,ttys_list[tty_i]), 2)


  if connected==False and all_conn and all_tty:
    # try IP
    mylcd.lcd_display_string("Try: 127.0.0.1:%s"%try_port,3)
    (conn, status) = try_connect(mylcd,'127.0.0.1:'+str(try_port))
    if status<>"connect":
      ip_fail=True
      if try_port>max_try_port:
        ty_port=min_try_port
      else:
        try_port+=1
    else:
      connected=True
      mylcd.lcd_display_string("OK: 127.0.0.1:%s"%try_port,4)
      time.sleep(2)
      # Connection successful!!
      break

# Now start displaying stuff  

# set delta thresholds
yaw_thresh = 1.0
lat_thresh = 0.000005 #0.00001
lon_thresh = lat_thresh
alt_thresh = 0.1
heartbeat_thresh_warn = 5
heartbeat_thresh_lost = 20

# stored values (last time display was updated)
yaw_save = 0.0
yaw_deg =  0.0
lat_save = 0.0
lon_save = 0.0
alt_save = 0.0
last_heartbeat = 0.0

# set if position needs to be updated
pos_dirty=0

print "Got vehicle" % vehicle
vehicle.flush()

mylcd.lcd_clear()

# Wait for a message
while vehicle.heartbeat_lastreceived==0: # and not api.exit:
  print "Waiting for connection"
  mylcd.lcd_display_string("Waiting             ",2)
  time.sleep(0.5)
  mylcd.lcd_display_string("             Waiting",2)
  time.sleep(0.5)

mylcd.lcd_clear()

# Get all vehicle attributes (state)
print "\nGet all vehicle attribute values:"
print " Mode: %s" % vehicle.mode.name    # settable
print " Armed: %s" % vehicle.armed    # settable

print " Location: %s" % vehicle.location
print " Attitude: %s" % vehicle.attitude
print " Velocity: %s" % vehicle.velocity
print " GPS: %s" % vehicle.gps_0
print " Groundspeed: %s" % vehicle.groundspeed
print " Airspeed: %s" % vehicle.airspeed


print " Mode: %s" % vehicle.mode.name    # settable
print " Armed: %s" % vehicle.armed    # settable

bl_on=True

while True: #not api.exit:
  #try:
  if True:
    # Check if yaw has changed
    orient_dirty=0
    yaw_deg=vehicle.attitude.yaw * 180.0 / 3.14159
    if abs(yaw_deg-yaw_save)>1 :
      yaw_save=yaw_deg
      orient_dirty=1
    if abs(vehicle.location.alt-alt_save)>alt_thresh:
      orient_dirty=1
      alt_save=vehicle.location.alt
      clear_line(mylcd,3)
      mylcd.lcd_display_string('Hdg:'+str(int(yaw_deg))+'   ',3)
      pos_str = "  Alt:"+str(round(alt_save,3))
      mylcd.lcd_display_string_pos(pos_str,3,9)
      time.sleep(0.05)
 
    # Check if position has changed
    pos_dirty=0
    if abs(vehicle.location.lat-lat_save)>lat_thresh:
      pos_dirty=1
      lat_save=vehicle.location.lat
    if abs(vehicle.location.lon-lon_save)>lon_thresh:
      pos_dirty=1
      lon_save=vehicle.location.lon
    if pos_dirty==1:
      mylcd.backlight(1)
      pos_str = "Lat:"+str(round(lat_save,6))+" "
      mylcd.lcd_display_string(pos_str,1)
      pos_str = "Lon:"+str(round(lon_save,6))+"  "
      mylcd.lcd_display_string(pos_str,2)
      time.sleep(0.05)

    # Print RSSI
    # Check heartbeat

    if (time.time()-vehicle.heartbeat_lastreceived)>heartbeat_thresh_warn:
      clear_line(mylcd,4)
      bh_str = "LINK LOST: "+str(int(time.time()-vehicle.heartbeat_lastreceived))
      mylcd.lcd_display_string(bh_str,4)
      bl_on=not bl_on
      if bl_on:
        mylcd.backlight(1)
      else:
        mylcd.backlight(0)
    else:
      rs_str = "RSS:%s, Rem:%s" % (round(vehicle.radiostatus.rssi,2), round(vehicle.radiostatus.remrssi,2))
      mylcd.lcd_display_string(rs_str,4)
    time.sleep(0.05)
##  except APIException as ae:
    if (time.time()-vehicle.heartbeat_lastreceived)>heartbeat_thresh_lost:
#    if vehicle.heartbeat_timeout:
      mylcd.lcd_display_string("**CONNECTION LOST**",4)
      while True:
        (conn, status) = try_connect(mylcd,'127.0.0.1:'+str(try_port))
        if status<>"connect":
          ip_fail=True
          #If we've tried all the ports, list the ttys
          if try_port>max_try_port:
            try_port=min_try_port
            ttys_list=find_ttys()
            for tty in ttys_list:
              clear_line(mylcd,3)
              mylcd.lcd_display_string(tty, 3)
              time.sleep(2)
          else:
            try_port+=1
        else:
          connected=True
          mylcd.lcd_display_string("OK: 127.0.0.1:%s"%try_port,4)
          # Connection successful!!
          break


vehicle.close()
 
 
