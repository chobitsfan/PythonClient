#Copyright © 2018 Naturalpoint
#
#Licensed under the Apache License, Version 2.0 (the "License");
#you may not use this file except in compliance with the License.
#You may obtain a copy of the License at
#
#http://www.apache.org/licenses/LICENSE-2.0
#
#Unless required by applicable law or agreed to in writing, software
#distributed under the License is distributed on an "AS IS" BASIS,
#WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#See the License for the specific language governing permissions and
#limitations under the License.


# OptiTrack NatNet direct depacketization sample for Python 3.x
#
# Uses the Python NatNetClient.py library to establish a connection (by creating a NatNetClient),
# and receive data via a NatNet connection and decode it using the NatNetClient library.

import os
os.environ['FOR_DISABLE_CONSOLE_CTRL_HANDLER'] = '1'
from NatNetClient import NatNetClient
from pymavlink import mavutil
import time, threading
from scipy import signal as scipy_signal
from collections import deque
import socket, signal

class Drone():
    def __init__(self):
        self.last_send_ts = 0
        self.last_sys_time = 0
        self.tracked = False
        self.master = None

drones = [ Drone() for i in range(10) ]
gogogo = True

def signal_handler(sig, frame):
    global gogogo
    gogogo = False

# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receiveRigidBodyFrame( id, position, rotation, trackingValid ):
    cur_ts = time.time()
    if trackingValid:
        #print( "Received frame for rigid body", id , position, rotation )
        x=position[0]
        y=position[2]
        z=-position[1]
        rot=(rotation[3], rotation[0], rotation[2], -rotation[1])
        drone = drones[id]
        if not drone.tracked:
            drone.tracked = True
            print(id, "tracked", cur_ts)
        if cur_ts - drone.last_send_ts > 0.04:
            if drone.last_send_ts == 0:
                drone.master = mavutil.mavlink_connection(device="udpout:192.168.0."+str(id)+":14550", source_system=255)
            drone.master.mav.att_pos_mocap_send(int(cur_ts * 1000000), rot, x, y, z)
            drone.last_send_ts = cur_ts
    else:
        drone = drones[id]
        if drone.tracked:
            drone.tracked = False
            print(id, "not tracked", cur_ts)

def main():
    signal.signal(signal.SIGINT, signal_handler)

    start_ts = time.time()

    # This will create a new NatNet client
    streamingClient = NatNetClient()

    # Configure the streaming client to call our rigid body handler on the emulator to send data out.
    streamingClient.newFrameListener = None
    streamingClient.rigidBodyListener = receiveRigidBodyFrame

    # Start up the streaming client now that the callbacks are set up.
    # This will run perpetually, and operate on a separate thread.
    streamingClient.run()

    while gogogo:
        for drone in drones:
            if drone.last_send_ts > 0:
                msg = drone.master.recv_msg()
                if msg is not None:
                    msg_type = msg.get_type()
                    if msg_type == "BAD_DATA":
                        print ("bad [", ":".join("{:02x}".format(c) for c in msg.get_msgbuf()), "]")
                    else:
                        if msg_type == "HEARTBEAT":
                            print ("[", msg.get_srcSystem(),"] heartbeat", time.time(), "mode", msg.custom_mode)
                        elif msg_type == "STATUSTEXT":
                            print ("[", msg.get_srcSystem(),"]", msg.text)
                        elif msg_type == "TIMESYNC":
                            if msg.tc1 > 0:
                                print ("[", msg.get_srcSystem(),"] latency ", (time.time() * 1000000 - msg.ts1) / 2000, " ms")

                cur_ts = time.time()
                if cur_ts - drone.last_sys_time > 13:
                    drone.master.mav.system_time_send(int(cur_ts * 1000000), int((cur_ts - start_ts)*1000))
                    drone.last_sys_time = cur_ts

    print("bye")

if __name__ == '__main__':
    main()