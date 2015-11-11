import time
import sys
import os
from dronekit import connect, VehicleMode
from dronekit.test import with_sitl
from nose.tools import assert_equals

@with_sitl
def test_110(connpath):
    vehicle = connect(connpath, wait_ready=True)

    # NOTE these are *very inappropriate settings*
    # to make on a real vehicle. They are leveraged
    # exclusively for simulation. Take heed!!!
    vehicle.parameters['FS_GCS_ENABLE'] = 0
    vehicle.parameters['FS_EKF_THRESH'] = 100
    
    # Change the vehicle into STABILIZE mode
    vehicle.mode = VehicleMode("STABILIZE")

    # NOTE wait crudely for ACK on mode update
    time.sleep(3)

    # Define example callback for mode
    def armed_callback(vehicle, attribute, value):
        armed_callback.called += 1
    armed_callback.called = 0

    # When the same (event, callback) pair is passed to add_attribute_listener,
    # only one instance of the observer callback should be added.
    vehicle.add_attribute_listener('armed', armed_callback)
    vehicle.add_attribute_listener('armed', armed_callback)
    vehicle.add_attribute_listener('armed', armed_callback)
    vehicle.add_attribute_listener('armed', armed_callback)
    vehicle.add_attribute_listener('armed', armed_callback)

    # Disarm and see update.
    vehicle.armed = False
    # Wait for ACK.
    time.sleep(3)

    # Ensure the callback was called.
    assert armed_callback.called > 0, "Callback should have been called."

    # Rmove all listeners. The first call should remove all listeners
    # we've added; the second call should be ignored and not throw.
    # NOTE: We test if armed_callback were treating adding each additional callback
    # and remove_attribute_listener were removing them one at a time; in this
    # case, there would be three callbacks still attached.
    vehicle.remove_attribute_listener('armed', armed_callback)
    vehicle.remove_attribute_listener('armed', armed_callback)
    callcount = armed_callback.called

    # Re-arm and see update.
    vehicle.armed = True
    # Wait for ack
    time.sleep(3)

    # Ensure the callback was called zero times.
    assert_equals(armed_callback.called, callcount, "Callback should not have been called once removed.")

    vehicle.close()
