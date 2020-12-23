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

from NatNetClient import NatNetClient
from pymavlink import mavutil
import time, threading
from scipy import signal
from collections import deque
import socket

class Drone():
    def __init__(self):
        self.posx = []
        self.posy = []
        self.posz = []
        self.last_send_ts = 0
        self.tracked = False

drones = [ Drone() for i in range(20) ] 
master = mavutil.mavlink_connection(device="udpout:192.168.0.10:14550", source_system=255)
master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receiveRigidBodyFrame( id, position, rotation, trackingValid ):
    sampling_period = 1.0/240.0
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
        drone.posx.append(x)
        drone.posy.append(y)
        drone.posz.append(z)        
        if cur_ts - drone.last_send_ts > 0.02:
            if len(drone.posx) < 5:
                #make sure vel is sent in the beginning
                if drone.last_send_ts == 0:
                    return
                print("not enough pos, no vel")
                velx = None
            else:
                velx = signal.savgol_filter(drone.posx, 5, 2, deriv=1, delta=sampling_period)
                vely = signal.savgol_filter(drone.posy, 5, 2, deriv=1, delta=sampling_period)
                velz = signal.savgol_filter(drone.posz, 5, 2, deriv=1, delta=sampling_period)
                drone.posx.clear()
                drone.posy.clear()
                drone.posz.clear()
            m = master.mav.att_pos_mocap_encode(int(cur_ts * 1000000), rot, x, y, z)
            m.pack(master.mav)
            b1 = m.get_msgbuf()
            if velx is None:
                #print("send pos");
                master.write(b1)
            else:
                #print("send pos and vel");
                m = master.mav.vision_speed_estimate_encode(int(cur_ts * 1000000), velx[-3], vely[-3], velz[-3])
                m.pack(master.mav)
                b2 = m.get_msgbuf()
                master.write(b2+b1)
            drone.last_send_ts = cur_ts
    else:
        drone = drones[id]
        if drone.tracked:
            drone.tracked = False
            print(id, "not tracked", cur_ts)
        drone.posx.clear()
        drone.posy.clear()
        drone.posz.clear()

def main():
    last_sys_time = 0
    lsat_hb_sent = 0
    start_ts = time.time()

    # This will create a new NatNet client
    streamingClient = NatNetClient()

    # Configure the streaming client to call our rigid body handler on the emulator to send data out.
    streamingClient.newFrameListener = None
    streamingClient.rigidBodyListener = receiveRigidBodyFrame

    # Start up the streaming client now that the callbacks are set up.
    # This will run perpetually, and operate on a separate thread.
    streamingClient.run()

    #sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    #sock.settimeout(0.005)
    #sock.bind(("127.0.0.1", 17500))
    #sock_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    while threading.active_count() > 1:    
        msg = master.recv_msg()
        if msg is not None:
            msg_type = msg.get_type()
            if msg_type == "BAD_DATA":
                print ("bad [", ":".join("{:02x}".format(c) for c in msg.get_msgbuf()), "]")
            else:
                if msg_type == "HEARTBEAT":
                    print ("[", msg.get_srcSystem(),"] heartbeat", time.time(), "mode", msg.custom_mode)
                elif msg_type == "STATUSTEXT":
                    print ("[", msg.get_srcSystem(),"]", msg.text)
                #elif msg_type == "LOCAL_POSITION_NED":
                #    print ("[", msg.get_srcSystem(),"]", msg.x, msg.y, msg.z)
                #sock_out.sendto(msg.get_msgbuf(), ("127.0.0.1", 17501))

        cur_ts = time.time()
        if cur_ts - last_sys_time > 5:
            master.mav.system_time_send(int(cur_ts * 1000000), int((cur_ts - start_ts)*1000))
            last_sys_time = cur_ts
        if cur_ts - lsat_hb_sent > 2:
            master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            lsat_hb_sent = cur_ts

        #try:
        #    data = sock.recv(512)
        #except socket.timeout:
        #    pass
        #else:
            #print("msg from unity", time.time(), len(data))
        #    master.write(data)

if __name__ == '__main__':
    main()