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
My first test with python and crazyflie
"""
import sys
sys.path.append("../lib")
import cflib.crtp
import logging
import time
from threading import Thread, Timer
import cflib.crtp
from cfclient.utils.logconfigreader import LogConfig
from cflib.crazyflie import Crazyflie
# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

class MyFirstExample:
    """MyExamples"""
    def __init__(self, link_uri):
        """ Initialize and run the example with the specified link_uri """

        self._cf = Crazyflie()

        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)
        self._cf.open_link(link_uri)
        # Variable used to keep main loop occupied until disconnect
        #self.is_connected = True
        print "Connecting to %s" % link_uri

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        #setup log config for sensor
        print("connected")
        #self._cf.commander.send_setpoint(0, 0, 0, 32767)
        self._log_sensor = LogConfig("Proximity sensor", 100)
        self._log_sensor.add_variable("adc.vProx")
        self._log_sensor.add_variable("baro.aslLong")
        self._log_sensor.add_variable("stabilizer.thrust")
        self._cf.log.add_config(self._log_sensor)

        if self._log_sensor.valid:
             # This callback will receive the data
            self._log_sensor.data_received_cb.add_callback(self._log_data)
            # This callback will be called on errors
            self._log_sensor.error_cb.add_callback(self._log_error)
            # Start the logging
            self._log_sensor.start()

        else:
            print("Could not add logconfig since some variables are not in TOC")

        Thread(target=self._hover).start()

    def _hover(self):

        thrust = 10767
        thrust_step = 10
        pitch = 0
        roll = 0
        yawrate = 0

        while thrust < 32767:
            self._cf.commander.send_setpoint(roll, pitch, yawrate, thrust)
            thrust += thrust_step
        self._cf.param.set_value("flightmode.althold", "True")
        while 0 < 1:
            self._cf.commander.send_setpoint(roll, pitch, yawrate, 32767)
            time.sleep(0.1)

        #    time.sleep(0.1)
        #print ("next")
        # Start a timer to disconnect in 5s
        #t = Timer(10, self._cf.close_link)
        #t.start()


        # Start a separate thread to do the motor test.
        # Do not hijack the calling thread!

        #Thread(target=self._ramp_motors).start()
        #self._ramp_motors()

    def _log_data(self, timestamp, data, logconf):
        """Callback froma the log API when data arrives"""

        global myprox
        global switch
        global ref_baro
        #global act_baro
        #global thrust

        thrust = data["stabilizer.thrust"]
        myprox = data["adc.vProx"]
        act_baro = data["baro.aslLong"]

        if switch is True:
            print("Switch")
            switch = False
            ref_baro = data["baro.aslLong"]

        print(thrust)
        #self._ramp_motors(thrust, act_baro)
        #long = float(data["adc.vProx"])
        #self._ramp_motors()
        #Thread(target=self._ramp_motors(thrust, act_baro)).start()
        #print "[%d][%s]: %s" % (timestamp, logconf.name, data)

    def _log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print "Error when logging %s: %s" % (logconf.name, msg)

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the speficied address)"""
        print "Connection to %s failed: %s" % (link_uri, msg)
        #self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        self._disconnected(link_uri)
        print "Connection to %s lost: %s" % (link_uri, msg)

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""

        print "Disconnected from %s...lANDING" % link_uri

        time.sleep(0.1)
        self._cf.close_link()
        #self.is_connected = False


    def _ramp_motors(self, thrust, act_baro):

        #thrust_mult = 1
        thrust_step = 10
        pitch = 0
        roll = 0
        yawrate = 0
        temp_thrust = thrust
        #thrust = 32767
        #print(thrust)

        #while thrust <= 32000:
        #    thrust += thrust_step
        #    self._cf.commander.send_setpoint(roll, pitch, yawrate, thrust)
      #     print(thrust, temp_thrust)
            #time.sleep(1)
        #self._ramp_motors(thrust, act_baro)
        #time.sleep(0.5)

        #if thrust > 30000:
        #if thrust > 0:
        #self._cf.param.set_value("flightmode.althold", "True")
        #     print("H O V E R I N G")
        #    self._cf.commander.send_setpoint(0, 0, 0, 32767)
        #    hover = True
        #self._ramp_motors(thrust,act_baro)

        # Make sure that the last packet leaves before the link is closed
        # since the message queue is not flushed before closing

        # H O V E R


        #time.sleep(0.5);



       # while some_condition:
       #     cf.commander.send_setpoint(0,0,0,32767);
       #     time.sleep(0.01);



if __name__ == '__main__':
    global switch
    switch = True
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    # Scan for Crazyflies and use the first one found
    print "Scanning interfaces for Crazyflies..."
    available = cflib.crtp.scan_interfaces()
    print "Crazyflies found:"
    for i in available:
        print i[0]
    if len(available) > 0:
        le = MyFirstExample(available[0][0])
        print("STAAAARTTT")
    else:
        print "No Crazyflies found, cannot run program"
