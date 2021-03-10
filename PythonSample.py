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
        self.time_offset = 0
        self.last_send_ts = 0
        self.last_sync_time = 0
        self.tracked = False
        self.master = None

drones = [ Drone() for i in range(10) ]
gogogo = True

def signal_handler(sig, frame):
    global gogogo
    gogogo = False

# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receiveRigidBodyFrame( id, position, rotation, trackingValid ):
    if trackingValid:
        #print( "Received frame for rigid body", id , position, rotation )
        x=position[0]
        y=position[2]
        z=-position[1]
        rot=(rotation[3], rotation[0], rotation[2], -rotation[1])
        drone = drones[id]
        cur_ts = time.time()
        if not drone.tracked:
            drone.tracked = True
            print(id, "tracked", cur_ts)
        if drone.master is None:
            drone.master = mavutil.mavlink_connection(device="udpout:192.168.50."+str(id+10)+":14550", source_system=255)
        if drone.time_offset == 0 and cur_ts - drone.last_sync_time > 3:
            drone.master.mav.system_time_send(int(cur_ts * 1000000), 0) # ardupilot ignore time_boot_ms 
            drone.last_sync_time = cur_ts
        if drone.time_offset > 0 and cur_ts - drone.last_send_ts > 0.03:
            drone.master.mav.att_pos_mocap_send(int(cur_ts * 1000000 - drone.time_offset), rot, x, y, z)
            drone.last_send_ts = cur_ts
    else:
        drone = drones[id]
        if drone.tracked:
            drone.tracked = False
            print(id, "not tracked", time.time())

def main():
    signal.signal(signal.SIGINT, signal_handler)

    # This will create a new NatNet client
    streamingClient = NatNetClient()

    # Configure the streaming client to call our rigid body handler on the emulator to send data out.
    streamingClient.newFrameListener = None
    streamingClient.rigidBodyListener = receiveRigidBodyFrame

    # Start up the streaming client now that the callbacks are set up.
    # This will run perpetually, and operate on a separate thread.
    streamingClient.run()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)
    sock.bind(("127.0.0.1", 17500))

    #drones[0].master = mavutil.mavlink_connection(device="udpout:192.168.0.2:14550", source_system=255)
    #drones[0].master.mav.system_time_send(int(time.time() * 1000000), 0) # ardupilot ignore time_boot_ms 
    #drones[0].last_sync_time = time.time()

    while gogogo:
        try:
            data, addr = sock.recvfrom(1024)
        except socket.error:
            pass
        else:
            drones[addr[1]-17500].master.write(data)
        for idx, drone in enumerate(drones):
            if drone.last_sync_time > 0:
                msg = drone.master.recv_msg()
                if msg is not None:
                    msg_type = msg.get_type()
                    if msg_type == "BAD_DATA":
                        print ("bad [", ":".join("{:02x}".format(c) for c in msg.get_msgbuf()), "]")
                    else:
                        sock.sendto(msg.get_msgbuf(), ("127.0.0.1", 17500+idx))
                        if msg_type == "HEARTBEAT":
                            print ("[", msg.get_srcSystem(),"] heartbeat", time.time(), "mode", msg.custom_mode)
                        elif msg_type == "STATUSTEXT":
                            print ("[", msg.get_srcSystem(),"]", msg.text)
                        elif msg_type == "TIMESYNC": #and drone.time_offset == 0:
                            if msg.tc1 == 0:
                                cur_us = time.time() * 1000000
                                drone.master.mav.timesync_send(int(cur_us), msg.ts1) # ardupilot ignore tc1
                                drone.time_offset = cur_us - msg.ts1 * 0.001 # ardupilot send ts1 amd tc1 in nano-seconds
                                print ("[", msg.get_srcSystem(),"] timesync", msg.ts1)
                                drone.master.mav.set_gps_global_origin_send(0, 247749434, 1210443077, 100000)
                            #drone.master.mav.command_long_send(0, 0, 511, 0, 31, 10000, 0, 0, 0, 0, 0)
                            #drone.master.mav.command_long_send(0, 0, 511, 0, 32, 10000, 0, 0, 0, 0, 0)
                        #elif msg_type == "ATTITUDE_QUATERNION":
                        #    cur_ts = time.time()
                        #    print("att interval", (cur_ts-att_ts)*1000)
                        #    att_ts=cur_ts
                        else:
                            print("[", msg.get_srcSystem(),"]", msg_type);

    print("bye")

if __name__ == '__main__':
    main()