.. _vehicle-information:

===========================
Vehicle State and Settings
===========================

The :py:class:`Vehicle <dronekit.lib.Vehicle>` class exposes *most* state information (position, speed, etc.) through python 
:ref:`attributes <vehicle_state_attributes>`, while vehicle :ref:`parameters <vehicle_state_parameters>` (settings) 
are accessed though named elements of :py:attr:`Vehicle.parameters <dronekit.lib.Vehicle.parameters>`. 

This topic explains how to get, set and observe vehicle state and parameter information (including getting the 
:ref:`Home location <vehicle_state_home_location>`).

.. tip:: You can test most of the code in this topic by running the :ref:`Vehicle State <example-vehicle-state>` example.


.. _vehicle_state_attributes:

Attributes
==========

Vehicle state information is exposed through vehicle *attributes*. DroneKit-Python currently supports the following 
"standard" attributes: 
:py:attr:`Vehicle.location.global_frame <dronekit.lib.Vehicle.location.global_frame>`, 
:py:attr:`Vehicle.location.local_frame <dronekit.lib.Vehicle.location.local_frame>`, 
:py:attr:`Vehicle.attitude <dronekit.lib.Vehicle.attitude>`,
:py:attr:`Vehicle.velocity <dronekit.lib.Vehicle.velocity>`,
:py:attr:`Vehicle.airspeed <dronekit.lib.Vehicle.airspeed>`,
:py:attr:`Vehicle.groundspeed <dronekit.lib.Vehicle.groundspeed>`,
:py:attr:`Vehicle.gps_0 <dronekit.lib.Vehicle.gps_0>`,
:py:attr:`Vehicle.mount_status <dronekit.lib.Vehicle.mount_status>`,
:py:attr:`Vehicle.battery <dronekit.lib.Vehicle.battery>`,
:py:attr:`Vehicle.rangefinder <dronekit.lib.Vehicle.rangefinder>`,
:py:attr:`Vehicle.ekf_ok <dronekit.lib.Vehicle.ekf_ok>`,
:py:attr:`Vehicle.home_location <dronekit.lib.Vehicle.home_location>`,
:py:func:`Vehicle.system_status <dronekit.lib.Vehicle.system_status>`,
:py:func:`Vehicle.heading <dronekit.lib.Vehicle.heading>`,
:py:func:`Vehicle.is_armable <dronekit.lib.Vehicle.is_armable>`,
:py:attr:`Vehicle.armed <dronekit.lib.Vehicle.armed>`,
:py:attr:`Vehicle.mode <dronekit.lib.Vehicle.mode>`.

Attributes are initially created with ``None`` values for their members. In most cases the members are populated 
(and repopulated) as new MAVLink messages of the associated types are received from the vehicle. 

All of the attributes can be :ref:`read <vehicle_state_read_attributes>`, 
but only the :py:attr:`Vehicle.home_location <dronekit.lib.Vehicle.home_location>`, 
:py:attr:`Vehicle.mode <dronekit.lib.Vehicle.mode>` and 
:py:attr:`Vehicle.armed <dronekit.lib.Vehicle.armed>` 
status can be :ref:`written <vehicle_state_set_attributes>`.

Almost all of the attributes can be :ref:`observed <vehicle_state_observe_attributes>`.

The behaviour of :py:attr:`Vehicle.home_location <dronekit.lib.Vehicle.home_location>` is different 
from the other attributes, and is :ref:`discussed in its own section below <vehicle_state_home_location>`.

.. _vehicle_state_read_attributes:

Getting attributes
------------------

The code fragment below shows how to read and print almost all the attributes (values are
regularly updated from MAVLink messages sent by the vehicle).

.. code:: python
    
    # vehicle is an instance of the Vehicle class
    print "Global Location: %s" % vehicle.location.global_frame
    print "Local Location: %s" % vehicle.location.local_frame    #NED
    print "Attitude: %s" % vehicle.attitude
    print "Velocity: %s" % vehicle.velocity
    print "GPS: %s" % vehicle.gps_0
    print "Groundspeed: %s" % vehicle.groundspeed
    print "Airspeed: %s" % vehicle.airspeed
    print "Mount status: %s" % vehicle.mount_status
    print "Battery: %s" % vehicle.battery
    print "EKF OK?: %s" % vehicle.ekf_ok
    print "Rangefinder: %s" % vehicle.rangefinder
    print "Rangefinder distance: %s" % vehicle.rangefinder.distance
    print "Rangefinder voltage: %s" % vehicle.rangefinder.voltage
    print "Heading: %s" % vehicle.heading
    print "Is Armable?: %s" % vehicle.is_armable
    print "System status: %s" % vehicle.system_status.state
    print "Mode: %s" % vehicle.mode.name    # settable
    print "Armed: %s" % vehicle.armed    # settable


.. note::

    A value of ``None`` for an attribute member indicates that the value has not yet been populated from the vehicle.
    For example, before GPS lock :py:attr:`Vehicle.gps_0 <dronekit.lib.Vehicle.gps_0>` will return a 
    :py:class:`GPSInfo <dronekit.lib.GPSInfo>` with ``None`` values for ``eph``, ``satellites_visible`` etc.
    Attributes will also return  ``None`` if the associated hardware is not present on the connected device. 


.. tip::

    If you're using a :ref:`simulated vehicle <sitl_setup>` you can add support for optional hardware including
    `rangefinders <http://dev.ardupilot.com/using-sitl-for-ardupilot-testing/#adding_a_virtual_rangefinder>`_
    and `optical flow sensors <http://dev.ardupilot.com/using-sitl-for-ardupilot-testing/#adding_a_virtual_optical_flow_sensor>`_.

    
.. todo:: we need to be able to verify mount_status works/setup.



.. _vehicle_state_set_attributes:

Setting attributes
------------------

Only the :py:attr:`Vehicle.mode <dronekit.lib.Vehicle.mode>` and :py:attr:`Vehicle.armed <dronekit.lib.Vehicle.armed>` 
attributes can be written (``Vehicle.home_location`` is a special case, as :ref:`discussed below <vehicle_state_home_location>`).

The attributes are set by assigning a value:

.. code:: python

    #disarm the vehicle
    vehicle.armed = False


.. warning::

    Changing a value is **not guaranteed to succeed**. 
    For example, vehicle arming can fail if the vehicle doesn't pass pre-arming checks.

    While the autopilot does send information about the success (or failure) of the request, 
    this is `not currently handled by DroneKit <https://github.com/dronekit/dronekit-python/issues/114>`_.


Code should not assume that an attempt to set an attribute will succeed. The example code snippet below polls the attribute values
to confirm they have changed before proceeding.

.. code:: python
    
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.mode.name=='GUIDED' and not vehicle.armed and not api.exit:
        print " Getting ready to take off ..."
        time.sleep(1)
    


.. _vehicle_state_observe_attributes:

Observing attribute changes
---------------------------

You can observe any of the vehicle attributes and monitor for changes without the need for polling.

Listeners ("observer callback functions") are invoked differently based on the *type of observed attribute*. Attributes that represent sensor values or other
"streams of information" are updated whenever a message is received from the vehicle. Attributes which reflect vehicle
"state" are only updated when their values change (for example 
:py:func:`Vehicle.system_status <dronekit.lib.Vehicle.system_status>`,
:py:attr:`Vehicle.armed <dronekit.lib.Vehicle.armed>`, and
:py:attr:`Vehicle.mode <dronekit.lib.Vehicle.mode>`).

Callbacks are added using :py:func:`Vehicle.add_attribute_listener() <dronekit.lib.Vehicle.add_attribute_listener>` or the
:py:func:`Vehicle.on_attribute() <dronekit.lib.Vehicle.on_attribute>` decorator method. The main difference between these methods
is that only attribute callbacks added with :py:func:`Vehicle.add_attribute_listener() <dronekit.lib.Vehicle.add_attribute_listener>` 
can be removed (see :py:func:`remove_attribute_listener() <dronekit.lib.Vehicle.remove_attribute_listener>`). 

The ``observer`` callback function is invoked with the following arguments:
        
* ``self`` - the associated :py:class:`Vehicle`. This may be compared to a global vehicle handle 
  to implement vehicle-specific callback handling (if needed).
* ``attr_name`` - the attribute name. This can be used to infer which attribute has triggered
  if the same callback is used for watching several attributes.
* ``msg`` - the attribute value (so you don't need to re-query the vehicle object).

The code snippet below shows how to add (and remove) a callback function to observe changes
in :py:attr:`Vehicle.location.global_frame <dronekit.lib.Vehicle.location.global_frame>` using 
:py:func:`Vehicle.add_attribute_listener() <dronekit.lib.Vehicle.add_attribute_listener>`. 
The two second ``sleep()`` is required because otherwise the observer might be removed before the the 
callback is first run.


.. code-block:: python
   :emphasize-lines: 7
     
    #Callback to print the location in global frames. 'value' is the updated value
    def location_callback(self, attr_name, value):
        print "Location (Global): ", value 

        
    # Add a callback `location_callback` for the `global_frame` attribute.
    vehicle.add_attribute_listener('global_frame', location_callback)

    # Wait 2s so callback can be notified before the observer is removed
    time.sleep(2)

    # Remove observer - specifying the attribute and previously registered callback function
    vehicle.remove_message_listener('location', location_callback)

    
The example below shows how you can declare an attribute callback using the 
:py:func:`Vehicle.on_attribute() <dronekit.lib.Vehicle.on_attribute>` decorator function.


.. code-block:: python
   :emphasize-lines: 3,4

    last_rangefinder_distance=0

    @vehicle.on_attribute('rangefinder')
    def rangefinder_callback(self,attr_name):
        #attr_name not used here.
        global last_rangefinder_distance
        if last_rangefinder_distance == round(self.rangefinder.distance, 1):
            return
        last_rangefinder_distance = round(self.rangefinder.distance, 1)
        print " Rangefinder (metres): %s" % last_rangefinder_distance

.. note::

    The fragment above stores the result of the previous callback and only prints the output when there is a 
    signficant change in :py:attr:`Vehicle.rangefinder <dronekit.lib.Vehicle.rangefinder>`. You might want to
    perform caching like this to ignore updates that are not significant to your code.
        
The examples above show how you can monitor a single attribute. You can pass the special name ('``*``') to specify a 
callback that will be called for any/all attribute changes:

.. code-block:: python

    # Demonstrate getting callback on any attribute change
    def wildcard_callback(self, attr_name, value):
        print " CALLBACK: (%s): %s" % (attr_name,value)

    print "\nAdd attribute calback detecting any attribute change"     
    vehicle.add_attribute_listener('*', wildcard_callback)


    print " Wait 1s so callback invoked before observer removed"
    time.sleep(1)

    print " Remove Vehicle attribute observer"    
    # Remove observer added with `add_attribute_listener()`
    vehicle.remove_attribute_listener('*', wildcard_callback) 



.. _vehicle_state_home_location:

Home location
-------------

The *Home location* is set when a vehicle first gets a good location fix from the GPS. The location is used 
as the target when the vehicle does a "return to launch". In Copter missions (and often Plane) missions, the altitude of 
waypoints is set relative to this position.

:py:attr:`Vehicle.home_location <dronekit.lib.Vehicle.home_location>` has the following behaviour:

* In order to *get* the current value (in a :py:class:`LocationGlobal <dronekit.lib.LocationGlobal>` object) you must first download 
  :py:attr:`Vehicle.commands <dronekit.lib.Vehicle.commands>`, as shown:

  .. code:: python
    
      cmds = vehicle.commands
      cmds.download()
      cmds.wait_ready()
      print " Home Location: %s" % vehicle.home_location

  The returned value is ``None`` before you download the commands or if the ``home_location`` has not yet been set by the autopilot.
  For this reason our example code checks that the value exists (in a loop) before writing it.
  
  .. code:: python
    
      # Get Vehicle Home location - will be `None` until first set by autopilot
      while not vehicle.home_location:
          cmds = vehicle.commands
          cmds.download()
          cmds.wait_ready()
          if not vehicle.home_location:
              print " Waiting for home location ..."
              
      # We have a home location.     
      print "\n Home location: %s" % vehicle.home_location

* The attribute can be *set* to a :py:class:`LocationGlobal <dronekit.lib.LocationGlobal>` object 
  (the code fragment below sets it to the current location):

  .. code:: python
    
        vehicle.home_location=vehicle.location.global_frame
        
  There are some caveats:
  
  * You must be able to read a non-``None`` value before you can write it
    (the autopilot has to set the value initially before it can be written or read).
  * The new location must be within 50 km of the EKF origin or setting the value will silently fail.
  * The value is cached in the ``home_location``. If the variable can potentially change on the vehicle
    you will need to re-download the ``Vehicle.commands`` in order to confirm the value.
    
* The attribute is not observable.

 
.. note::

    :py:attr:`Vehicle.home_location <dronekit.lib.Vehicle.home_location>` behaves this way because
    ArduPilot implements/stores the home location as a waypoint rather than sending them as messages. 
    While DroneKit-Python hides this fact from you when working with commands, to access the value
    you still need to download the commands.
    
    We hope to improve this attribute in later versions of ArduPilot, where there may be specific 
    commands to get the home location from the vehicle.


.. _vehicle_state_parameters:

Parameters
==========

Vehicle parameters provide the information used to configure the autopilot for the vehicle-specific hardware/capabilities.
The available parameters for each platform are documented in the ardupilot wiki here:  
`Copter Parameters <http://copter.ardupilot.com/wiki/configuration/arducopter-parameters/>`_, 
`Plane Parameters <http://plane.ardupilot.com/wiki/arduplane-parameters/>`_, 
`Rover Parameters <http://rover.ardupilot.com/wiki/apmrover2-parameters/>`_ 
(the lists are automatically generated from the latest ArduPilot source code, and may contain or omit parameters
in your vehicle).

DroneKit downloads all parameters when you first connect to the UAV (forcing parameter reads to wait 
until the download completes), and subsequently keeps the values updated by monitoring vehicle messages 
for changes to individual parameters. This process ensures that it is always safe to read supported parameters, 
and that their values will match the information on the vehicle.

Parameters can be read and set using the :py:attr:`Vehicle.parameters <dronekit.lib.Vehicle.parameters>` 
attribute (a :py:class:`Parameters <dronekit.lib.Parameters>` object).


Getting parameters
------------------

The parameters are read using the parameter name as a key. Reads will always succeed unless you attempt to access an unsupported parameter
(which will result in a ``KeyError`` exception).
   
The code example below shows how to set Minimum Throttle (THR_MIN) setting. On Copter and Rover (not Plane), this is the minimum PWM setting for the 
throttle at which the motors will keep spinning.

.. code:: python

    # Print the value of the THR_MIN parameter.
    print "Param: %s" % vehicle.parameters['THR_MIN']



Setting parameters
------------------

Vehicle parameters are set as shown in the code fragment below, using the parameter name as a "key":

.. code:: python

    # Change the parameter value (Copter, Rover)
    vehicle.parameters['THR_MIN']=100


Observing parameter changes
---------------------------

At time of writing :py:class:`Parameters <dronekit.lib.Parameters>` does `not support <https://github.com/dronekit/dronekit-python/issues/107>`_ observing parameter changes.

.. todo:: 

    Check to see if observers have been implemented and if so, update the information here, in about, and in Vehicle class:
    https://github.com/dronekit/dronekit-python/issues/107




.. _api-information-known-issues:

Known issues
============

Below are a number of bugs and known issues related to vehicle state and settings:

* `#107 Add implementation for observer methods in Parameter class <https://github.com/dronekit/dronekit-python/issues/107>`_ 
* `#114 DroneKit has no method for detecting command failure <https://github.com/dronekit/dronekit-python/issues/114>`_


Other API issues and improvement suggestions can viewed on `github here <https://github.com/dronekit/dronekit-python/issues>`_. 