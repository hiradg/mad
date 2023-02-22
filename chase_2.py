import time
import math
from pymavlink import mavutil

target_sysID = 1
chaser_sysID = 2

k_p = 0.15
k_i = 0.0003
des_mode = 4
chaser_max_speed =20

connection_str = 'udp:127.0.0.1:14551'#'tcp:127.0.0.1:5773'# 'udp:172.18.208.1:14550'

control_loop_freq = 20        #Hz

vel_bitmask = 0b001111000111

class chaseController(object):
    def __init__(self,target_sysID,chaser_sysID,k_p,k_i,connection_str,control_loop_freq,des_mode,chaser_max_speed):

        self.master = mavutil.mavlink_connection(connection_str)
        print(self.master.wait_heartbeat())

        self.target_sysid = target_sysID
        self.chaser_sysid = chaser_sysID
        self.k_p = k_p
        self.k_i = k_i 
        self.control_loop_freq = control_loop_freq
        self.des_mode = des_mode
        self.chaser_max_speed = chaser_max_speed

        self.mode_flag = 0

        self.heading_target = 0.0
        self.heading_chaser = 0.0
        
        self.local_pose_target = [0,0,0]
        self.local_pose_chaser = [0,0,0]

        self.local_vel_target  = [0,0,0]
        self.local_vel_chaser  = [0,0,0]

        self.global_pose_target = [0,0,0]
        self.global_pose_chaser = [0,0,0]

        self.chase_target_point_offset = 25 # this the distance of the point in front of the USV (in meters) to chase 
        self.chase_target_alt_offset = -5
        self.chase_target_point = [0,0,0]

        self.chasePoint_x           = 0
        self.chasePoint_y           = 0
        self.chase_point_heading    = 0 

        self.hold_dist_xy   = -5
        self.hold_dist_z    = -5
        self.hold_point     = [0,0,0]

        self.distError_Integral_term = 0
        self.distErrorSign  = 0 # It should be -1 or 1 which will be set in the code 
        self.distError      = 0 # This is absolute distance between the USV and the UAV
        self.delta_N        = 0
        self.delta_E        = 0 

        self.type_mask_velocity = 0b110111000111 # use vel+yaw+yaw_rate and ignor pos+accel+forces
        self.type_mask_position = 0b110111111000 # used by the position_target message to tell the UAV to only use position
                                                 # Documentation at https://github.com/mavlink/mavlink/pull/978/files

        self.speed_feedForward_term  = 0
        self.speed_proportional_term = 0 
        self.speed_integral_term     = 0

        self.chaser_speed           = 0
        self.chaser_speed_goal      = 0
        self.chaser_speed_goal_x    = 0
        self.chaser_speed_goal_y    = 0
        self.chaser_speed_goal_z    = 0 

        self.target_speed             = 0

    def align_home_positions(self):
        self.request_msg(self.target_sysid, 242)
        target_pose = self.master.recv_match(type = 'HOME_POSITION', blocking=True)
        self.set_home_position(target_pose)
        #maybe check home position confirmation though should be able to see it on missionplanner

    def set_chase_target_pose(self):

        self.get_target_local_pose()
        self.get_target_heading()

        #set chase point infront of plane
        chase_target_x = self.local_pose_target[0]-self.chase_target_point_offset*math.cos(math.radians(self.heading_target))
        chase_target_y = self.local_pose_target[1]-self.chase_target_point_offset*math.sin(math.radians(self.heading_target))
        chase_target_z = self.local_pose_target[2]-self.chase_target_alt_offset

        self.chase_target_point=[chase_target_x,chase_target_y,chase_target_z]
        print('chase target pose')
        print(self.chase_target_point)
        print('target pose')
        print(self.local_pose_target)

    def set_hold_point(self):
        hold_point_x = self.local_pose_target[0]-math.cos(math.radians(self.heading_target))*(self.hold_dist_xy)
        hold_point_y = self.local_pose_target[1]-math.sin(math.radians(self.heading_target))*(self.hold_dist_xy)
        hold_point_z = self.local_pose_target[2]-self.hold_dist_z
        self.hold_point =[hold_point_x,hold_point_y,hold_point_z]
        print('hold point')
        print(self.hold_point)

    def get_target_heading(self):
        self.request_msg(self.target_sysid, 33)
        target_global_pose = self.master.recv_match(type = 'GLOBAL_POSITION_INT',blocking = True)
        if target_global_pose.get_srcSystem() == self.target_sysid:
            self.heading_target = target_global_pose.hdg

    def request_msg(self,target_system,msg_id):#request any message
        self.master.mav.command_long_send(
        target_system,
        self.master.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
        0,
        msg_id, 0, 0, 0, 0, 0, 0
        )
        
    def set_home_position(self,target_home_pose):
        self.master.mav.command_long_send( #set home position on chaser drone 
        self.chaser_sysid,
        self.master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
        0,
        0, 0, 0, 0, target_home_pose.latitude/10000000.0, target_home_pose.longitude/10000000.0, target_home_pose.altitude/1000.0
        )
    
    def get_chaser_local_pose(self):
        self.request_msg(self.chaser_sysid, 32)
        local_pose = self.master.recv_match(type = 'LOCAL_POSITION_NED',blocking=True)
        if local_pose.get_srcSystem() == self.chaser_sysid:
            self.local_pose_chaser=[local_pose.x,local_pose.y,local_pose.z]
            self.local_vel_chaser=[local_pose.vx,local_pose.vy,local_pose.vz]
            self.chaser_speed = math.sqrt(local_pose.vx**2+local_pose.vy**2)
            print('chaser local pose')
            print(local_pose)

    def get_target_local_pose(self):
        self.request_msg(self.target_sysid,32)
        local_pose = self.master.recv_match(type = 'LOCAL_POSITION_NED',blocking=True)
        if local_pose.get_srcSystem() == self.target_sysid:
            # print(local_pose)
            self.local_pose_target=[local_pose.x,local_pose.y,local_pose.z]
            self.local_vel_target=[local_pose.vx,local_pose.vy,local_pose.vz]
            self.target_speed = math.sqrt(local_pose.vx**2+local_pose.vy**2)
            print('target local pose')
            print(local_pose)
    
    def get_dist_error(self): #gets dist error and integral term.
        self.delta_E = self.local_pose_chaser[1]-self.hold_point[1]
        self.delta_N = self.local_pose_chaser[0]-self.hold_point[0]

        self.get_error_sign(self.delta_N,self.delta_N)

        self.distError = math.sqrt(self.delta_N**2+self.delta_E**2)

        self.distError_Integral_term = self.distError_Integral_term+self.distError*self.distErrorSign*1/self.control_loop_freq

        print('dist error')
        print(self.distError)
        print(self.delta_N)
        print(self.delta_E)
    
    def get_error_sign(self,delta_N,delta_E):

        y_target_frame =  math.sin((self.heading_target))*(delta_N)+math.cos((self.heading_target))*(delta_E)

        if y_target_frame>0:
            self.distErrorSign = 1
        elif y_target_frame<=0:
            self.distErrorSign = -1
        print('error sign')
        print(self.distErrorSign)

    def get_heading_to_chase_point(self):
        delta_E = self.local_pose_chaser[1]-self.chase_target_point[1]
        delta_N = self.local_pose_chaser[0]-self.chase_target_point[0]

        self.chase_point_heading = math.atan2(delta_N,delta_E)
        print('chase point heading')

    def get_heading_to_hold_point(self):
        delta_E = self.local_pose_chaser[1]-self.hold_point[1]
        delta_N = self.local_pose_chaser[0]-self.hold_point[0]

        self.chase_point_heading = math.atan2(delta_N,delta_E)
        print('chase point heading')
        print(self.chase_point_heading)

    def vel_controller_horizontal(self):
        self.chaser_speed_goal = self.target_speed*self.distErrorSign + self.k_p*self.distError*self.distErrorSign+self.k_i*self.distError_Integral_term

        if self.chaser_speed_goal> self.chaser_max_speed:
            self.chaser_speed_goal = self.chaser_max_speed

        self.chaser_speed_goal_x = self.chaser_speed_goal*math.sin((self.chase_point_heading))
        self.chaser_speed_goal_y = self.chaser_speed_goal*math.cos((self.chase_point_heading))
        print('horizontal vel controller')
        print(self.target_speed)
        print(self.chaser_speed_goal)
        print(self.chaser_speed_goal_x)
        print(self.chaser_speed_goal_y)

    def vel_controller_vertical(self):
        delta_z = self.chase_target_point[2]-self.local_pose_chaser[2]

        vert_speed_goal = self.local_vel_target[2]+self.k_p*delta_z
        self.chaser_speed_goal_z = vert_speed_goal

        
    def send_vel_command(self):
        self.master.mav.set_position_target_local_ned_send(0,#ms since boot
        self.chaser_sysid,
        self.master.target_component,
        1,
        self.type_mask_velocity,
        self.chase_target_point[0],
        self.chase_target_point[1],
        self.chase_target_point[2],
        self.chaser_speed_goal_x,
        self.chaser_speed_goal_y,
        self.chaser_speed_goal_z,
        0,
        0,
        0,
        self.heading_target,
        0
        )
        print('velcoity commands')
        print(self.chaser_speed_goal_x)
        print(self.chaser_speed_goal_y)
        print(self.chaser_speed_goal_z)

    def check_flight_mode(self):#make this bit work
        self.request_msg(self.chaser_sysid,0)
        msg = self.master.recv_match(type = 'HEARTBEAT',blocking=True)
        src = msg.get_srcSystem()
        if src == self.chaser_sysid:
            mode_id = msg.custom_mode
            if mode_id == self.des_mode:
                self.mode_flag = 1
            else:
                self.mode_flag = 0
        print('check flight mode')

        

    def control_loop(self):
        
        #check mode
        self.check_flight_mode()

        while self.mode_flag == 1:
            self.set_chase_target_pose() #set target pose
            self.set_hold_point()
            self.get_chaser_local_pose() 
            print("chaser pose")
            print(self.local_pose_chaser)#get current chaser pose
            self.get_heading_to_hold_point()
            self.get_dist_error()
            self.vel_controller_horizontal()
            self.vel_controller_vertical()
            self.send_vel_command()    
            self.check_flight_mode()
            time.sleep(1/self.control_loop_freq)
            

        time.sleep(1/self.control_loop_freq)

def main():
    controller = chaseController(target_sysID,chaser_sysID,k_p,k_i,connection_str,control_loop_freq,des_mode,chaser_max_speed)
    controller.align_home_positions()
    time.sleep(1)#allow home posiitons to align
    #maybe add some in put logic to start after home alignment checked
    while True:
        controller.control_loop()
        print('running loop')

main()
