# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2014 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.

#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.

"""
Simple example that builds upon the basiclog.py and ramp.py, demonstarte 
how to build a feedback control loop that runs at 10 Hz for 5 seconds
has to be run in the example folder just like the other examples
"""

import sys
#modify here to make the code runnable somewhere else
sys.path.append("../lib")

import cflib.crtp

import logging
import time
from threading import Timer
from threading import Lock
from threading import Thread

import cflib.crtp
from cfclient.utils.logconfigreader import LogConfig
from cflib.crazyflie import Crazyflie

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


class LogItem:
    def __init__(self):
	self.zacc = 0;
	self.baro = 0;
	self.time = 0;

    def __init__(self, zacc, baro, time):
	self.zacc = zacc;
	self.baro = baro;
	self.time = time;

    def __str__(self):
	return "acc in z: {0}, baro pressure: {1}, at time {2}\n".format(self.zacc,self.baro,self.time)

class ControlInput:
    def __init__(self, roll, pitch, yawrate,thrust):
        self.roll = roll
	self.pitch = pitch
	self.yawrate = yawrate
	self.thrust = thrust

class LogKeeper:

    def __init__(self):
        self._log = []
	self._lock = Lock()

    def addItem(self,time, data):
	#the lock here ensures data synchornizing between read and write
	with self._lock:
            zacc = data['acc.z']
	    baro = data['baro.pressure']
	    temp = LogItem(zacc, baro, time)
	    self._log.append(temp)

    def getItem(self):
        #the lock here ensures data synchornizing between read and write
	with self._lock:
	    if self._log:
	    	print "current state: " + str(self._log[-1])
	    	return self._log[-1]
	    else:
		return LogItem(0,0,0)

    def __str__(self):
	result = ""
	for item in self._log:
	    result = result + str(item)
	return result

def feedbackControl(currentState):
    #filling your own controller
    thrust = 23000
    pitch = 0
    roll = 0
    yawrate = 0

    return ControlInput(roll, pitch, yawrate, thrust)

	
class LoggingExample:
    """
    Simple logging example class that logs the Stabilizer from a supplied
    link uri and disconnects after 5s.
    """
    def __init__(self, link_uri):
        """ Initialize and run the example with the specified link_uri """

        # Create a Crazyflie object without specifying any cache dirs
        self._cf = Crazyflie()

        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        print "Connecting to %s" % link_uri

        # Try to connect to the Crazyflie
        self._cf.open_link(link_uri)

        # Variable used to keep main loop occupied until disconnect
        self.is_connected = True
	
	self._logKeeper = LogKeeper();

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print "Connected to %s" % link_uri

        # The definition of the logconfig can be made before connecting
        self._lg_stab = LogConfig(name="Stabilizer", period_in_ms=10)
        self._lg_stab.add_variable("acc.z", "float")
	self._lg_stab.add_variable("baro.pressure", "float")

        # Adding the configuration cannot be done until a Crazyflie is
        # connected, since we need to check that the variables we
        # would like to log are in the TOC.
        self._cf.log.add_config(self._lg_stab)
        if self._lg_stab.valid:
            # This callback will receive the data
            self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
            # This callback will be called on errors
            self._lg_stab.error_cb.add_callback(self._stab_log_error)
            # Start the logging
            self._lg_stab.start()
        else:
            print("Could not add logconfig since some variables are not in TOC")

        # Start a timer to disconnect in 5s
        t = Timer(50, self._cf.close_link)
        t.start()
	
	Thread(target=self._pid_z).start()

    def _pid_z(self):
	while (self.is_connected):
	    currentState = self._logKeeper.getItem()
	    #add the feedback controller here
	    controlInput = feedbackControl(currentState)
	    self._cf.commander.send_setpoint(controlInput.roll, controlInput.pitch, controlInput.yawrate, controlInput.thrust)
            
	    #sending a constant cmd
	    #self._cf.commander.send_setpoint(0, 0, 0, 20000)
	    #defines the control frequency, can be set to time.sleep(0.01), which is 100 Hz
            time.sleep(0.1)

        self._cf.commander.send_setpoint(0, 0, 0, 0)


    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print "Error when logging %s: %s" % (logconf.name, msg)

    def _stab_log_data(self, timestamp, data, logconf):
        """Callback froma the log API when data arrives"""
        #print "[%d][%s]: %s" % (timestamp, logconf.name, data)
	self._logKeeper.addItem(timestamp, data)

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the speficied address)"""
        print "Connection to %s failed: %s" % (link_uri, msg)
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print "Connection to %s lost: %s" % (link_uri, msg)

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print "Disconnected from %s" % link_uri
        self.is_connected = False

if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    # Scan for Crazyflies and use the first one found
    print "Scanning interfaces for Crazyflies..."
    available = cflib.crtp.scan_interfaces()
    print "Crazyflies found:"
    for i in available:
        print i[0]

    if len(available) > 0:
        le = LoggingExample(available[0][0])
    else:
        print "No Crazyflies found, cannot run example"

    # The Crazyflie lib doesn't contain anything to keep the application alive,
    # so this is where your application should do something. In our case we
    # are just waiting until we are disconnected.
    while le.is_connected:
        time.sleep(1)
	
    if not (le.is_connected):
	print le._logKeeper