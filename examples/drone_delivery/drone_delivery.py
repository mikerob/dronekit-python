"""
drone_delivery.py: 

A CherryPy based web application that displays a mapbox map to let you view the current vehicle position and send the vehicle commands to fly to a particular latitude and longitude.

Full documentation is provided at http://python.dronekit.io/examples/drone_delivery.html
"""

import os
import simplejson
import time
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobal
import cherrypy
from cherrypy.process import wspbus, plugins
from jinja2 import Environment, FileSystemLoader

#Set up option parsing to get connection string
import argparse  
parser = argparse.ArgumentParser(description='Print out vehicle state information. Connects to SITL on local PC by default.')
parser.add_argument('--connect', default='127.0.0.1:14550',
                   help="vehicle connection target. Default '127.0.0.1:14550'")
args = parser.parse_args()



local_path=os.path.dirname(os.path.abspath(__file__))
print "local path: %s" % local_path


cherrypy_conf = {
    '/': {
        'tools.sessions.on': True,
        'tools.staticdir.root': local_path
    },
    '/static': {
        'tools.staticdir.on': True,
        'tools.staticdir.dir': './html/assets'
    }
}


class Drone(object):
    def __init__(self, home_coords, server_enabled=True):
        self.gps_lock = False
        self.altitude = 30.0

        # Connect to the Vehicle
        self._log('Connected to vehicle.')
        self.vehicle = vehicle
        self.commands = self.vehicle.commands
        self.current_coords = []
        self.home_coords = home_coords
        self.webserver_enabled = server_enabled
        self._log("DroneDelivery Start")

        # Register observers
        self.vehicle.add_attribute_listener('location', self.location_callback)

    def launch(self):
        self._log("Waiting for ability to arm...")
        while not self.vehicle.is_armable:
            time.sleep(.1)

        self._log('Running initial boot sequence')
        self.change_mode('GUIDED')
        self.arm()
        self.takeoff()

        if self.webserver_enabled is True:
            self._run_server()

    def takeoff(self):
        self._log("Taking off")
        self.commands.takeoff(30.0)
        self.vehicle.flush()
        
    def arm(self, value=True):
        if value:
            self._log('Waiting for arming...')
            self.vehicle.armed = True    
            while not self.vehicle.armed:
                time.sleep(.1)
        else:
            self._log("Disarming!")
            self.vehicle.armed = False

    def _run_server(self):
        # Start web server if enabled
        cherrypy.tree.mount(DroneDelivery(self), '/', config=cherrypy_conf)

        cherrypy.config.update({
            'server.socket_port': 8080,
            'server.socket_host': '0.0.0.0',
            'log.screen': None
         })

        print 'http://localhost:8080/'
        cherrypy.engine.start()

    def change_mode(self, mode):
        self._log("Changing to mode: {0}".format(mode))

        self.vehicle.mode = VehicleMode(mode)
        while self.vehicle.mode.name != mode:
            self._log('  ... polled mode: {0}'.format(mode))
            time.sleep(1)

    def goto(self, location, relative=None):
        self._log("Goto: {0}, {1}".format(location, self.altitude))

        self.commands.goto(
            LocationGlobal(
                float(location[0]), float(location[1]),
                float(self.altitude),
                is_relative=relative
            )
        )
        self.vehicle.flush()

    def get_location(self):
        return [self.current_location.lat, self.current_location.lon]

    def location_callback(self, vehicle, name, location):
        if location.alt is not None:
            self.altitude = location.alt

        self.current_location = location

    def _log(self, message):
        print "[DEBUG]: {0}".format(message)

class Templates:
    def __init__(self, home_coords):
        self.home_coords = home_coords
        self.options = self.get_options()
        self.environment = Environment(loader=FileSystemLoader( local_path + '/html'))

    def get_options(self):
        return {
                'width': 670,
                'height': 470,
                'zoom': 13,
                'format': 'png',
                'access_token': 'pk.eyJ1IjoibXJwb2xsbyIsImEiOiJtUG0tRk9BIn0.AqAiefUV9fFYRo-w0jFR1Q',
                'mapid': 'mrpollo.kfbnjbl0',
                'home_coords': self.home_coords,
                'menu': [
                    {'name': 'Home', 'location': '/'},
                    {'name': 'Track', 'location': '/track'},
                    {'name': 'Command', 'location': '/command'}
                    ],
                'current_url': '/',
                'json': ''
                }

    def index(self):
        self.options = self.get_options()
        self.options['current_url'] = '/'
        return self.get_template('index')

    def track(self, current_coords):
        self.options = self.get_options()
        self.options['current_url'] = '/track'
        self.options['current_coords'] = current_coords
        self.options['json'] = simplejson.dumps(self.options)
        return self.get_template('track')

    def command(self, current_coords):
        self.options = self.get_options()
        self.options['current_url'] = '/command'
        self.options['current_coords'] = current_coords
        return self.get_template('command')

    def get_template(self, file_name):
        template = self.environment.get_template( file_name + '.html')
        return template.render(options=self.options)

class DroneDelivery(object):
    def __init__(self, drone):
        self.drone = drone
        self.templates = Templates(self.drone.home_coords)

    @cherrypy.expose
    def index(self):
        return self.templates.index()

    @cherrypy.expose
    def command(self):
        return self.templates.command(self.drone.get_location())

    @cherrypy.expose
    @cherrypy.tools.json_out()
    def vehicle(self):
        return dict(position=self.drone.get_location())

    @cherrypy.expose
    def track(self, lat=None, lon=None):
        # Process POST request from Command
        # Sending MAVLink packet with goto instructions
        if(lat is not None and lon is not None):
            self.drone.goto([lat, lon], True)

        return self.templates.track(self.drone.get_location())


# Connect to the Vehicle
print 'Connecting to vehicle on: %s' % args.connect
vehicle = connect(args.connect, wait_ready=True)

Drone([32.5738, -117.0068]).launch()
cherrypy.engine.block()
