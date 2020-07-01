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

posx = []
posy = []
posz = []
sampling_period = 1.0/120.0
last_pos_send_ts = 0
uwb_anchor = mavutil.mavlink_connection(device="com3", baud=3000000, source_system=255)

# This is a callback function that gets connected to the NatNet client and called once per mocap frame.
def receiveNewFrame( frameNumber, markerSetCount, unlabeledMarkersCount, rigidBodyCount, skeletonCount,
                    labeledMarkerCount, timecode, timecodeSub, timestamp, isRecording, trackedModelsChanged ):
    print( "Received frame", frameNumber )

# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receiveRigidBodyFrame( id, position, rotation ):
    global last_pos_send_ts
    #print( "Received frame for rigid body", id , position, rotation)
    x=position[0]
    y=position[2]
    z=-position[1]
    rot=(rotation[3], rotation[0], rotation[2], -rotation[1])
    posx.append(x)
    posy.append(y)
    posz.append(z)
    cur_ts = time.time()
    if cur_ts - last_pos_send_ts > 0.07 and len(posx) > 5:
        velx = signal.savgol_filter(posx, 5, 2, deriv=1, delta=sampling_period)
        vely = signal.savgol_filter(posy, 5, 2, deriv=1, delta=sampling_period)
        velz = signal.savgol_filter(posz, 5, 2, deriv=1, delta=sampling_period)
        posx.clear()
        posy.clear() 
        posz.clear() 
        cur_us = int(cur_ts * 1000000)
        m = uwb_anchor.mav.att_pos_mocap_encode(1, 0, cur_us, rot, x, y, z)        
        m.pack(uwb_anchor.mav)
        b1 = m.get_msgbuf()
        m = uwb_anchor.mav.vision_speed_estimate_encode(cur_us, velx[-3], vely[-3], velz[-3])
        m.pack(uwb_anchor.mav)
        b2 = m.get_msgbuf()
        uwb_anchor.write(b2+b1)
        last_pos_send_ts = time.time()

# This will create a new NatNet client
streamingClient = NatNetClient()

# Configure the streaming client to call our rigid body handler on the emulator to send data out.
streamingClient.newFrameListener = None
streamingClient.rigidBodyListener = receiveRigidBodyFrame

# Start up the streaming client now that the callbacks are set up.
# This will run perpetually, and operate on a separate thread.
streamingClient.run()

while threading.active_count() > 1:    
    msg = uwb_anchor.recv_msg()
    if msg is not None:
        msg_type = msg.get_type()
        if msg_type == "BAD_DATA":
            print ("bad [", ":".join("{:02x}".format(c) for c in msg.get_msgbuf()), "]")
            pass
        else:
            if msg_type == "HEARTBEAT":
                print ("[", msg.get_srcSystem(),"] heartbeat", time.time(), "mode", msg.custom_mode)
            elif msg_type == "STATUSTEXT":
                print ("[", msg.get_srcSystem(),"]", msg.text)