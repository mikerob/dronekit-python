"""
vehicle_state.py: 

Demonstrates how to get and set vehicle state, parameter and channel-override information, 
and how to observe vehicle attribute (state) changes.

Full documentation is provided at http://python.dronekit.io/examples/vehicle_state.html
"""
from dronekit import connect
#import droneapi.lib
import RPi_I2C_driver
from droneapi.lib import VehicleMode, Vehicle
from pymavlink import mavutil
import time, subprocess, sys


# Run a shell command (like ls or grep)
def run_cmd(cmd):
  p = subprocess.Popen(cmd, shell= True, stdout=subprocess.PIPE)
  (output, err) = p.communicate()
  return output

# Mirror MAVLink Radio Status
class MAVlink_Radio_Status:
  rssi = -99.9
  remrssi = 99.9
  txbuf = 100
  noise = -66.6
  remnoise = 66.6
  rxerrors = 1
  fixed = 2


# Initialise display
mylcd = RPi_I2C_driver.lcd()

mylcd.lcd_clear()
mylcd.lcd_display_string("Booting up",1)

# Find viable connections

dev_tty_usb = run_cmd("ls /dev/ | grep ttyUSB")
dev_tty_acm = run_cmd("ls /dev/ | grep ttyA")
dev_tty = ['']

if len(dev_tty_usb)>0:
  print "No USB connections"
  dev_tty = dev_tty+dev_tty_usb[0].split("\n")
"""if len(dev_tty_acm)>0:
  print "There are %s A* connections", len(dev_tty_acm)
  dev_tty = dev_tty+dev_tty_acm.split("\n")
  """
print "Full:"+str(dev_tty)

print "USB:"+dev_tty_usb
print "ACM:"+dev_tty_acm


# Connect to the Vehicle
#api=local_connect()
"""for tty_x in dev_tty: 
  if tty_x<>"":
    dev_tty_str="/dev/"+tty_x
    print dev_tty_str
    try:
    vehicle = Vehicle()
    
    vehicle = connect(dev_tty_str)
    time.sleep(2)
    print vehicle.mode
    print "Pitch:",vehicle.pitch"""
vehicle=connect('127.0.0.1:14551')    	
#    except:
#      print "Bad: %s, Exception: %s"% (dev_tty_str, sys.exc_info()[0])

#print "\n Vehicles connected: %s" % len(api.get_vehicles())
#vehicle = api.get_vehicles()[0]

print vehicle

print "Got vehicle" % vehicle
vehicle.flush()



# set delta thresholds
yaw_thresh = 1.0
lat_thresh = 0.000005 #0.00001
lon_thresh = lat_thresh
alt_thresh = 0.1
heartbeat_thresh = 5

# stored values (last time display was updated)
yaw_save = 0.0
yaw_deg =  0.0
lat_save = 0.0
lon_save = 0.0
alt_save = 0.0
last_heartbeat = 0.0

# set if position needs to be updated
pos_dirty=0

# Demo callback handler for raw MAVLink messages

time_save = 0
time_ms = 0
# Set up RADIO_STATUS empty structure
rad_stat = MAVlink_Radio_Status()


# Set global to value of "time_boot_ms"
def mavrx_debug_handler(message):
  m_id = message.get_msgId()
  if m_id == 0: # HEARTBEAT
    global last_heartbeat
    last_heartbeat = time.time()
  elif m_id == 1: # SYS_STATUS
    #print message
    batt = message.voltage_battery
  elif m_id==2: # SYSTEM_TIME
    global time_save
    time_save=int(message.time_boot_ms)
  elif m_id==109: # RADIO_STATUS
    global rad_stat_msg
    rad_stat.rssi = message.rssi
    rad_stat.remrssi = message.remrssi
    rad_stat.txbuf = message.txbuf
    rad_stat.noise = message.noise
    rad_stat.remnoise = message.remnoise
    rad_stat.rxerrors = message.rxerrors
    rad_stat.fixed = message.fixed

# Set MAVLink callback handler (after getting Vehicle instance)
vehicle.set_mavlink_callback(mavrx_debug_handler)

# Wait for a message
while last_heartbeat==0: # and not api.exit:
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

while True: #not api.exit:
  # Check if yaw has changed
  yaw_deg=vehicle.attitude.yaw * 180.0 / 3.14159
  if abs(yaw_deg-yaw_save)>1 :
    yaw_save=yaw_deg
    mylcd.lcd_display_string('Hdg:'+str(int(yaw_deg))+'  ',1)
    time.sleep(0.1)

  # Check if position has changed
  pos_dirty=0
  if abs(vehicle.location.lat-lat_save)>lat_thresh:
    pos_dirty=1
    lat_save=vehicle.location.lat
  if abs(vehicle.location.lon-lon_save)>lon_thresh:
    pos_dirty=1
    lon_save=vehicle.location.lon
  if abs(vehicle.location.alt-alt_save)>alt_thresh:
    pos_dirty=1
    alt_save=vehicle.location.alt
  if pos_dirty==1:
    pos_str = "Lat:"+str(round(lat_save,6))+" "
    mylcd.lcd_display_string(pos_str,2)
    pos_str = "Lon:"+str(round(lon_save,6))+"  "
    mylcd.lcd_display_string(pos_str,3)
    pos_str = "Alt:"+str(round(alt_save,1))
    mylcd.lcd_display_string_pos(pos_str,1,11)
    time.sleep(0.1)
  
  # Print latest time
  #ms_str = "Time:" + str(time_save)
  #mylcd.lcd_display_string(ms_str,4)
  #time.sleep(0.25)

  # Print RSSI
  # Check heartbeat
  if (time.time()-last_heartbeat)>heartbeat_thresh:
    bh_str = "                ****"
    mylcd.lcd_display_string(bh_str,4)
    bh_str = "LINK LOST: "+str(int(time.time()-last_heartbeat))
    mylcd.lcd_display_string(bh_str,4)
  else:
    rs_str = "RSS:%s, Rem:%s" % (round(rad_stat.rssi,2), round(rad_stat.remrssi,2))
    mylcd.lcd_display_string(rs_str,4)
  time.sleep(0.1)

print "\nRemoving observer"
#vehicle.remove_attribute_observer('location', location_callback)
#vehicle.unset_mavlink_callback()

mylcd.lcd_display_string("**** TERMINATED ****",4)





