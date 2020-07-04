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
sampling_period = 1.0/120.0
uwb_anchor = mavutil.mavlink_connection(device="com3", baud=3000000, source_system=255)
uwb_last_send_ts = 0
msgs = deque()

# This is a callback function that gets connected to the NatNet client and called once per mocap frame.
def receiveNewFrame( frameNumber, markerSetCount, unlabeledMarkersCount, rigidBodyCount, skeletonCount,
                    labeledMarkerCount, timecode, timecodeSub, timestamp, isRecording, trackedModelsChanged ):
    print( "Received frame", frameNumber )

# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receiveRigidBodyFrame( id, position, rotation, trackingValid ):    
    if trackingValid:
        #print( "Received frame for rigid body", id , position, rotation )
        x=position[0]
        y=position[2]
        z=-position[1]
        rot=(rotation[3], rotation[0], rotation[2], -rotation[1])
        drone = drones[id]
        if not drone.tracked:
            drone.tracked = True
            print(id, "tracked")
        drone.posx.append(x)
        drone.posy.append(y)
        drone.posz.append(z)
        cur_ts = time.time()
        if cur_ts - drone.last_send_ts > 0.07:
            if len(drone.posx) < 5:
                velx = None
            else:
                velx = signal.savgol_filter(drone.posx, 5, 2, deriv=1, delta=sampling_period)
                vely = signal.savgol_filter(drone.posy, 5, 2, deriv=1, delta=sampling_period)
                velz = signal.savgol_filter(drone.posz, 5, 2, deriv=1, delta=sampling_period)
            drone.posx.clear()
            drone.posy.clear() 
            drone.posz.clear() 
            cur_us = int(cur_ts * 1000000)
            m = uwb_anchor.mav.att_pos_mocap_encode(id, 0, cur_us, rot, x, y, z)        
            m.pack(uwb_anchor.mav)
            b1 = m.get_msgbuf()
            if velx is None:
                msgs.append(b1)
            else:
                m = uwb_anchor.mav.vision_speed_estimate_encode(id, 0, cur_us, velx[-3], vely[-3], velz[-3])
                m.pack(uwb_anchor.mav)
                b2 = m.get_msgbuf()
                msgs.append(b2+b1)
            drone.last_send_ts = cur_ts
    else:
        drone = drones[id]
        if drone.tracked:
            drone.tracked = False
            print(id, "not tracked")
        drone.posx.clear()

# This will create a new NatNet client
streamingClient = NatNetClient()

# Configure the streaming client to call our rigid body handler on the emulator to send data out.
streamingClient.newFrameListener = None
streamingClient.rigidBodyListener = receiveRigidBodyFrame

# Start up the streaming client now that the callbacks are set up.
# This will run perpetually, and operate on a separate thread.
streamingClient.run()

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(0.001)
sock.bind(("127.0.0.1", 17500))
while threading.active_count() > 1:
    if len(msgs) > 0:
        uwb_anchor.write(msgs.popleft())
    try:
        data = sock.recv(512)
    except socket.timeout:
        pass
    else:
        uwb_anchor.write(data)
    