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

#import os
#os.environ['FOR_DISABLE_CONSOLE_CTRL_HANDLER'] = '1'
from NatNetClient import NatNetClient
from pymavlink import mavutil
import time, socket, select

print("hello")

all_mygcs_ip = []
#drone_network = "192.168.50."
#try:
#    with open('drone_net.txt','r') as f:
#        drone_network = f.read().strip()
#except FileNotFoundError:
#    pass
#print('drone network [', drone_network,']')

class Drone():
    def __init__(self, id):
        #self.time_offset = 0 # in micro-seconds
        self.hb_rcvd = False
        self.last_send_ts = 0
        self.id = id
        self.tracked = False
        #if id == 1:
        #self.master = mavutil.mavlink_connection(device="udpin:0.0.0.0:"+str(37500+id), source_system=255)
        self.master = mavutil.mavlink_connection(device="udpout:140.96.178.37:"+str(17501+id), source_system=255)
        self.pos = ()
        self.last_adsb_ts = 0
        self.last_debug_ts = 0
        self.lastPos = ()
        self.last_unity_send_ts = 0

DRONES_MAX_COUNT = 2
drones = [ Drone(i+1) for i in range(DRONES_MAX_COUNT) ]
local_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
local_sock.bind(("0.0.0.0", 17500))
game_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
game_sock.bind(("0.0.0.0", 27500))

#prvStampCameraExposure = 0

rigid_bodies = [ None ] * DRONES_MAX_COUNT

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

# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receiveRigidBodyFrame( id, position, rotation, trackingValid ):
    rigid_bodies[id-1] = (id, position, rotation, trackingValid)

# This is a callback function that gets connected to the NatNet client and called once per mocap frame.
def receiveNewFrame( frameNumber, markerSetCount, unlabeledMarkersCount, rigidBodyCount, skeletonCount,
                    labeledMarkerCount, timecode, timecodeSub, timestamp, stampCameraExposure, isRecording, trackedModelsChanged ):
    #stampCameraExposure Given in host's high resolution ticks, 1 tick = 0.1 us in my windows 10
    #timestamp given in seconds
    #global prvStampCameraExposure
    #print("recv frame", stampCameraExposure - prvStampCameraExposure)
    #prvStampCameraExposure = stampCameraExposure
    for rigid_body in rigid_bodies:
        id = rigid_body[0]
        trackingValid = rigid_body[3]
        drone = drones[id-1]
        if trackingValid:
            position = rigid_body[1]
            rotation = rigid_body[2]
            x=position[0]
            y=position[2]
            z=-position[1]
            drone.pos = (x,y,z)
            rot=(rotation[3], rotation[0], rotation[2], -rotation[1])
            if not drone.tracked:
                drone.tracked = True
                print(id, "tracked", stampCameraExposure * 0.0000001)
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

            #if stampCameraExposure - drone.last_unity_send_ts >= 150000:
            m = drone.master.mav.att_pos_mocap_encode(int(timestamp * 1000000), (rotation[3], rotation[0], rotation[1], rotation[2]), position[0], position[1], position[2])
            m.pack(drone.master.mav)
            for mygcs_ip in all_mygcs_ip:
                local_sock.sendto(m.get_msgbuf(), (mygcs_ip, 17500+id))
            #drone.last_unity_send_ts = stampCameraExposure

            # tom world only use althold, do not need viso pos
            #if drone.hb_rcvd:
            #    if stampCameraExposure - drone.last_send_ts >= 500000:
            #        if drone.lastPos:
            #            m = drone.master.mav.att_pos_mocap_encode(int(timestamp*1000000), rot, x, y, z) # time_usec
            #            m.pack(drone.master.mav)
            #            b = m.get_msgbuf()
            #            elapsed_time = stampCameraExposure - drone.last_send_ts
            #            m = drone.master.mav.vision_speed_estimate_encode(int(timestamp*1000000), (x-drone.lastPos[0])*10000000/elapsed_time, (y-drone.lastPos[1])*10000000/elapsed_time, (z-drone.lastPos[2])*10000000/elapsed_time)
            #            m.pack(drone.master.mav)
            #            drone.master.write(b+m.get_msgbuf())
            #        drone.lastPos = (x,y,z)
            #        drone.last_send_ts = stampCameraExposure

                # drone avoid is not used in ncsist
                # for opponent in drones:
                #     if opponent.tracked and opponent.id != drone.id and ((opponent.pos[0]-drone.pos[0])**2+(opponent.pos[1]-drone.pos[1])**2)<=0.64 and abs(opponent.pos[2]-drone.pos[2])<0.3:
                #         if cur_ts - drone.last_adsb_ts >= 0.02:
                #             v1 = qv_mult(rot, (1,0,0))
                #             v2 = (opponent.pos[0]-drone.pos[0],opponent.pos[1]-drone.pos[1])
                #             angle = math.atan2(v_2d_cross(v1,v2), v_2d_dot(v1,v2))
                #             if angle < 0:
                #                 angle = angle + 2 * math.pi
                #             sector = int((angle / (2 * math.pi / 16) + 1) / 2)
                #             if sector == 8:
                #                 sector = 0
                #             drone.master.mav.distance_sensor_send(int(cur_ts*1000-drone.time_offset*0.001),int(opponent.pos[0]*100+1000),int(opponent.pos[1]*100+1000),0,10,0,sector,0)
                #             drone.last_adsb_ts = cur_ts
                #             break
        else:
            if drone.tracked:
                drone.tracked = False
                print(id, "not tracked", stampCameraExposure * 0.0000001)

def main():
    # This will create a new NatNet client
    streamingClient = NatNetClient()

    # Configure the streaming client to call our rigid body handler on the emulator to send data out.
    streamingClient.newFrameListener = receiveNewFrame
    streamingClient.rigidBodyListener = receiveRigidBodyFrame

    # Start up the streaming client now that the callbacks are set up.
    # This will run perpetually, and operate on a separate thread.
    streamingClient.myinit()

    #drones[0].master = mavutil.mavlink_connection(device="udpout:192.168.0.2:14550", source_system=255)
    #drones[0].master.mav.system_time_send(int(time.time() * 1000000), 0) # ardupilot ignore time_boot_ms 
    #drones[0].last_sync_time = time.time()

    inputs = []
    for drone in drones:
        inputs.append(drone.master.port)
    inputs.append(local_sock)
    inputs.append(streamingClient.commandSocket)
    inputs.append(streamingClient.dataSocket)
    inputs.append(game_sock)

    last_hb_send_ts = 0

    while True:
        try:
            readables, writables, exceptionals = select.select(inputs, [], [], 10)
        except KeyboardInterrupt:
            break
        if local_sock in readables:
            try:
                data, addr = local_sock.recvfrom(1024)
            except socket.error as err:
                if err.errno != 10054:
                    print(err)
            else:
                if(len(data) > 0):
                    drones[addr[1]-17500-1].master.write(data)
        if game_sock in readables:
            try:
                data, addr = game_sock.recvfrom(1024)
            except socket.error as err:
                if err.errno != 10054:
                    print(err)
            else:
                if addr[0] not in all_mygcs_ip:
                    print("new client", addr)
                    all_mygcs_ip.append(addr[0])
                if(len(data) > 0):
                    for mygcs_ip in all_mygcs_ip:
                        if mygcs_ip != addr[0]:
                            game_sock.sendto(data, (mygcs_ip, addr[1]))
        if streamingClient.commandSocket in readables:
            try:
                data = streamingClient.commandSocket.recv( 32768 ) # 32k byte buffer size
            except socket.error as err:
                if err.errno != 10054:
                    print(err)
            else:
                if(len(data) > 0):
                    streamingClient.processMessage( data )
        if streamingClient.dataSocket in readables:
            try:
                data = streamingClient.dataSocket.recv( 32768 ) # 32k byte buffer size
            except socket.error as err:
                if err.errno != 10054:
                    print(err)
            else:
                if(len(data) > 0):
                    streamingClient.processMessage( data )
        for drone in drones:
            if drone.master.port in readables:
                while True:
                    try:
                        msg = drone.master.recv_msg()
                    except ConnectionResetError:
                        msg = None
                    if msg is None:
                        break
                    else:
                        msg_type = msg.get_type()
                        if msg_type == "BAD_DATA":
                            print ("bad [", ":".join("{:02x}".format(c) for c in msg.get_msgbuf()), "]")
                        else:
                            for mygcs_ip in all_mygcs_ip:
                                local_sock.sendto(msg.get_msgbuf(), (mygcs_ip, 17500+drone.id))
                            if msg_type == "HEARTBEAT":
                                print ("[", msg.get_srcSystem(),"] heartbeat", time.time(), "mode", msg.custom_mode, "seq", msg.get_seq())
                                drone.hb_rcvd = True
                            elif msg_type == "STATUSTEXT":
                                print ("[", msg.get_srcSystem(),"]", msg.text)
                            elif msg_type == "TIMESYNC":
                                if msg.tc1 == 0: # ardupilot send a timesync message every 10 seconds
                                    cur_us = int(time.time() * 1000000) # to micro-seconds
                                    drone.master.mav.timesync_send(cur_us, msg.ts1) # ardupilot log TSYN if tc1 != 0 and ts1 match
                                    #drone.time_offset = cur_us - msg.ts1 # I modified ardupilot send ts1 in us instead of nano-sec
                                    #print ("[", msg.get_srcSystem(),"] timesync", msg.ts1 / 1000000.0) # print in seconds

                                    #drone.master.mav.system_time_send(cur_us, 0) # ardupilot ignore time_boot_ms
                                    #drone.master.mav.set_gps_global_origin_send(0, 247749434, 1210443077, 100000)
                            else:
                                #print("[", msg.get_srcSystem(),"]", msg_type);
                                pass            
        cur_ts = time.time()
        if cur_ts - last_hb_send_ts > 1:
            last_hb_send_ts = cur_ts
            for drone in drones:
                drone.master.mav.heartbeat_send(6, 8, 0, 0, 0)

    print("bye")

if __name__ == '__main__':
    main()