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
import time, threading, socket, signal, select, math

class Drone():
    def __init__(self, id):
        self.time_offset = 0 # in micro-seconds
        self.last_send_ts = 0
        self.id = id
        self.tracked = False
        self.master = mavutil.mavlink_connection(device="udpout:192.168.50."+str(id+10)+":14550", source_system=255)
        self.pos = ()
        self.last_adsb_ts = 0
        self.last_debug_ts = 0

drones = [ Drone(i+1) for i in range(10) ]
gogogo = True

def q_conjugate(q):
    w, x, y, z = q
    return (w, -x, -y, -z)

def q_mult(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    return w, x, y, z

def qv_mult(q1, v1):
    q2 = (0.0,) + v1
    return q_mult(q_mult(q1, q2), q_conjugate(q1))[1:]

def v_2d_dot(v1, v2):
    return v1[0]*v2[0]+v1[1]*v2[1]

def v_2d_cross(v1, v2):
    return v1[0]*v2[1]-v1[1]*v2[0]

def signal_handler(sig, frame):
    global gogogo
    gogogo = False

# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receiveRigidBodyFrame( id, position, rotation, trackingValid ):
    drone = drones[id-1]
    if trackingValid:
        #print( "Received frame for rigid body", id , position, rotation )
        x=position[0]
        y=position[2]
        z=-position[1]
        drone.pos = (x,y,z)
        rot=(rotation[3], rotation[0], rotation[2], -rotation[1])
        cur_ts = time.time()
        if not drone.tracked:
            drone.tracked = True
            print(id, "tracked", cur_ts)
        #if drone.master is None:
        #    drone.master = mavutil.mavlink_connection(device="udpout:192.168.50."+str(id+10)+":14550", source_system=255)
        #if drone.time_offset == 0 and cur_ts - drone.last_sync_time > 3:
        #    drone.master.mav.system_time_send(int(cur_ts * 1000000), 0) # ardupilot ignore time_boot_ms 
        #    drone.last_sync_time = cur_ts

        # if cur_ts - drone.last_debug_ts > 1:
        #     drone.last_debug_ts = cur_ts
        #     print("heading", qv_mult(rot, (1,0,0)))
        #     v1 = qv_mult(rot, (1,0,0))
        #     for others in drones:
        #         if others.tracked and others.id != drone.id:
        #             v2 = (others.pos[0]-drone.pos[0],others.pos[1]-drone.pos[1])
        #             if drone.id == 2:
        #                 angle = math.atan2(v_2d_cross(v1,v2), v_2d_dot(v1,v2))
        #                 print("angle", angle*180.0/math.pi)
        #                 if angle < 0:
        #                     angle = angle + 2 * math.pi
        #                 sector = int((angle / (2 * math.pi / 16) + 1) / 2)
        #                 if sector == 8:
        #                     sector = 0
        #                 print("sector", sector)

        if drone.time_offset > 0:
            if cur_ts - drone.last_send_ts >= 0.03:
                drone.master.mav.att_pos_mocap_send(int(cur_ts * 1000000 - drone.time_offset), rot, x, y, z) # time_usec
                drone.last_send_ts = cur_ts
            for others in drones:
                if others.tracked and others.id != drone.id and ((others.pos[0]-drone.pos[0])**2+(others.pos[1]-drone.pos[1])**2)<=0.64 and abs(others.pos[2]-drone.pos[2])<0.3:
                    if cur_ts - drone.last_adsb_ts >= 0.02:
                        v1 = qv_mult(rot, (1,0,0))
                        v2 = (others.pos[0]-drone.pos[0],others.pos[1]-drone.pos[1])
                        angle = math.atan2(v_2d_cross(v1,v2), v_2d_dot(v1,v2))
                        if angle < 0:
                            angle = angle + 2 * math.pi
                        sector = int((angle / (2 * math.pi / 16) + 1) / 2)
                        if sector == 8:
                            sector = 0
                        drone.master.mav.distance_sensor_send(int(cur_ts*1000-drone.time_offset*0.001),0,0,0,10,0,sector,0)
                        drone.last_adsb_ts = cur_ts
                        break

    else:        
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

    local_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    local_sock.bind(("127.0.0.1", 17500))

    #drones[0].master = mavutil.mavlink_connection(device="udpout:192.168.0.2:14550", source_system=255)
    #drones[0].master.mav.system_time_send(int(time.time() * 1000000), 0) # ardupilot ignore time_boot_ms 
    #drones[0].last_sync_time = time.time()

    inputs = [local_sock]
    for drone in drones:
        inputs.append(drone.master.port)

    last_sys_time_sent = 0

    while gogogo:
        readables, writables, exceptionals = select.select(inputs, [], [], 10)
        for readable in readables:
            idx = inputs.index(readable)
            if idx == 0:
                try:
                    data, addr = readable.recvfrom(1024)
                except socket.error as err:
                    if err.errno != 10054:
                        print(err)
                else:
                    drones[addr[1]-17500-1].master.write(data)
            else:
                drone = drones[idx-1]
                try:
                    msg = drone.master.recv_msg()
                except ConnectionResetError:
                    msg = None
                if msg is not None:
                    msg_type = msg.get_type()
                    if msg_type == "BAD_DATA":
                        print ("bad [", ":".join("{:02x}".format(c) for c in msg.get_msgbuf()), "]")
                    else:
                        local_sock.sendto(msg.get_msgbuf(), ("127.0.0.1", 17500+idx))
                        if msg_type == "HEARTBEAT":
                            print ("[", msg.get_srcSystem(),"] heartbeat", time.time(), "mode", msg.custom_mode)
                        elif msg_type == "STATUSTEXT":
                            print ("[", msg.get_srcSystem(),"]", msg.text)
                        elif msg_type == "TIMESYNC":
                            if msg.tc1 == 0: # ardupilot send a timesync message every 10 seconds
                                cur_us = time.time() * 1000000 # to micro-seconds
                                drone.master.mav.timesync_send(int(cur_us), msg.ts1) # ardupilot log TSYN if tc1 != 0 and ts1 match
                                drone.time_offset = cur_us - msg.ts1 # I modified ardupilot send ts1 in us instead if ns
                                print ("[", msg.get_srcSystem(),"] timesync", msg.ts1 / 1000000.0) # print in seconds
                                drone.master.mav.set_gps_global_origin_send(0, 247749434, 1210443077, 100000)
                        else:
                            #print("[", msg.get_srcSystem(),"]", msg_type);
                            pass
        cur_ts = time.time()
        if cur_ts - last_sys_time_sent > 5:
            last_sys_time_sent = cur_ts
            for drone in drones:
                if drone.time_offset == 0:
                    #print("send system_time to drone", drone.id)
                    drone.master.mav.system_time_send(int(time.time() * 1000000), 0) # ardupilot ignore time_boot_ms 

    print("bye")

if __name__ == '__main__':
    main()