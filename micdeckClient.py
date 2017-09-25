import time
import numpy as np
from threading import Timer

import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie import Console

from continuousStream import ContinousStream
from streamPort import StreamPort

CF_ON_TIME = 300
SAMPLING_FREQ = 9500

class CrazyflieConnection:

    def __init__(self, link_uri):
        # Create a Crazyflie object without specifying any cache dirs
        self._cf = Crazyflie()

        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        # Displays uri of Crazyflie to connect
        print('Connecting to %s' % link_uri)

        # Try to connect to the Crazyflie
        self._cf.open_link(link_uri)

        # Variable used to keep main loop occupied until disconnect
        self.is_connected = True

    def _connected(self, link_uri):
        """This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print('Connected to %s' % link_uri)

        # Start a timer to disconnect in 10s
        t = Timer(CF_ON_TIME, self._cf.close_link)
        t.start()
        # Starts getting data from the stream port
        self.sp = StreamPort(cf_conn._cf, q, 29, CF_ON_TIME, SAMPLING_FREQ)

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the speficied address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)
        self.is_connected = False

def console_callback(text):
    print(text)

if __name__ == '__main__':
    # Displays graph
    cs = ContinousStream(4, SAMPLING_FREQ, 24)
    # Queue where data to be displayed is added
    q = cs.start()
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    # Scan for Crazyflies and use the first one found
    print('Scanning interfaces for Crazyflies...')
    available = cflib.crtp.scan_interfaces()

    print('Crazyflies found:')
    for i in available:
        print(i[0])

    # Connects to fist crazyflie in list
    if len(available) > 0:
        cf_conn = CrazyflieConnection(available[0][0])
    else:
        print('No Crazyflies found, cannot run example')
        cs.process.terminate()
        exit(-1)

    console = Console(cf_conn._cf)
    console.receivedChar.add_callback(console_callback)

    # From 0.1 to 0.1 sec checks if crazyflie is alive and the window is opened
    while cf_conn.is_connected & cs.process.is_alive():
        time.sleep(0.1)

    # Terminates continuous stream process when crazyflie disconnects or the window is closed
    if cf_conn.is_connected:
        cf_conn._cf.close_link()

    np.savetxt("new.csv", cf_conn.sp.audioVector[:cf_conn.sp.ptr], delimiter=",")

    cs.process.terminate()