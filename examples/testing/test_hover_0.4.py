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
#sys.path.append("../../lib")
sys.path.append("../../src")
sys.path.append("../../src/cflib")
import cflib.crtp
import logging
import time
from threading import Thread, Timer
from cfclient.utils.logconfigreader import LogConfig
from cflib.crazyflie import Crazyflie
# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

class HoverTest:
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
        self._log_sensor = LogConfig("Logs", 1000)
        self._log_sensor.add_variable("altHold.err")
        self._log_sensor.add_variable("altHold.target")

        #self._cf.log.add_config(self._log_sensor)

        try:
            self._cf.log.add_config(self._log_sensor)
            # This callback will receive the data
            self._log_sensor.data_received_cb.add_callback(self._log_data)
            # This callback will be called on errors
            self._log_sensor.error_cb.add_callback(self._log_error)
            # Start the logging
            self._log_sensor.start()
        except KeyError as e:
            print("Could not start log configuration,"
                  "{} not found in TOC".format(str(e)))
        except AttributeError:
            print("Could not add Stabilizer log config, bad configuration.")

        # Start a timer to disconnect in s
        print("Motors ON, starting timer")
         # Start a timer to disconnect in 10s


        t = Timer(5, self._landing)
        t.start()
        # Start a separate thread to do the motor test.
        # Do not hijack the calling thread!
        Thread(target=self._ramp_motors).start()

        #t = Timer(5, self._cf.close_link)


    def _log_data(self, timestamp, data, logconf):
        """Callback froma the log API when data arrives"""

        #global ref_baro
        #ref_baro = data["baro.aslLong"]
        #print(ref_baro)
        print "[%d][%s]: %s" % (timestamp, data, logconf.name)

    def _log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print "Error when logging %s: %s" % (logconf.name, msg)

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

    def _landing(self):
        thrust_mult = -1
        thrust_step = 400
        thrust = 20000
        global landing
        landing = True
        time.sleep(1)
        print "Landing ON"

        self._cf.commander.send_setpoint(0, 0, 0, 0)
        self._cf.param.set_value("flightmode.althold","False")

        while thrust >= 10000:
            self._cf.commander.send_setpoint(0, 0, 0, thrust)
            thrust += thrust_step * thrust_mult
            time.sleep(0.1)
        print ("Landing done")
        time.sleep(2)
        self._cf.close_link()


    def _ramp_motors(self):

        thrust = 15000
        global landing
        landing = False

        #print(ref_baro)

        # H O V E R
        # Unlock startup thrust protection
        self._cf.commander.send_setpoint(0, 0, 0, 0)
        print ("Starting motors thurst:",thrust)
        #self._cf.commander.send_setpoint(0,0,0,thrust)
        time.sleep(0.75)

        if landing == False:
            counter = 0
            #self._cf.param.set_value("flightmode.althold","True")
            #print "Hover ON"

        while landing == False:
            #thrust = 32767

            if counter == 0 :
               print ("Prepare to hover")
               self._cf.commander.send_setpoint(0,0,0,32767)

            if counter > 0:
                self._cf.commander.send_setpoint(0,0,0,32767)
                self._cf.param.set_value("flightmode.althold","True")
                print ("Hovering")

            counter = counter + 1
            #self._cf.commander.send_setpoint(0,0,0,32767)

            #time.sleep(0.1)

            time.sleep(1)
            print ("Sleep 1")

        print "althold OFF"
        self._cf.param.set_value("flightmode.althold","False")
        time.sleep(1)
        #self._cf.close_link()

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
        le = HoverTest(available[0][0])
    else:
        print "No Crazyflies found, cannot run program"

    #while le.is_connected:
    #    time.sleep(1)