import time
import math
from pymavlink import mavutil
from geometry import GPS, Point

target_sysID = 1
chaser_sysID = 2

k_p = 0.001
k_i = 0.0000
k_d = 0.0000

des_mode = 4
chaser_max_speed =20

connection_str = 'udp:127.0.0.1:14551'#'tcp:127.0.0.1:5773'# 'udp:172.18.208.1:14550'

control_loop_freq = 10        #Hz

class chaseController(object):
    def __init__(self,target_sysID,chaser_sysID,k_p,k_i,connection_str,control_loop_freq,des_mode,chaser_max_speed):

        self.master = mavutil.mavlink_connection(connection_str)
        print(self.master.wait_heartbeat())

        self.target_sysid = target_sysID
        self.chaser_sysid = chaser_sysID
        self.k_p = k_p
        self.k_i = k_i 
        self.k_d = k_d
        self.control_loop_freq = control_loop_freq
        self.desired_timestep = 1/control_loop_freq
        self.last_timestep = self.desired_timestep
        
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

        self.chase_target_point_offset = -25 # this the distance of the point in front of the USV (in meters) to chase 
        self.chase_target_alt_offset = 10
        self.chase_target_point = [0,0,0]

        self.chasePoint_x           = 0
        self.chasePoint_y           = 0
        self.chase_point_heading    = 0 

        self.hold_dist_xy   = 5
        self.hold_dist_z    = -5
        self.hold_point     = [0,0,0]

        self.distError_Integral_term = 0
        self.distErrorSign  = 0 # It should be -1 or 1 which will be set in the code 
        self.distError      = 0 # This is absolute distance between the USV and the UAV
        self.distError_I = 0
        self.distError_d = 0
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
        self._tb = 0
        
        self.z_error = 10
        
        self.last_time = time.time()
        
    def align_home_positions(self):
        home_flag =0
        while home_flag ==0:
            self.request_msg(self.target_sysid, 242)
            target_pose = self.master.recv_match(type = 'HOME_POSITION', blocking=True)
            if target_pose.get_srcSystem() == self.target_sysid:
                self.set_home_position(target_pose)
                home_flag = 1
        #maybe check home position confirmation though should be able to see it on missionplanner

    def set_chase_target_pose(self):

        self.get_target_local_pose()
        self.get_target_heading()

        #set chase point infront of plane
        chase_target_x = self.local_pose_target[0]-self.chase_target_point_offset*math.cos(math.radians(self.heading_target))
        chase_target_y = self.local_pose_target[1]-self.chase_target_point_offset*math.sin(math.radians(self.heading_target))
        chase_target_z = self.local_pose_target[2]-self.chase_target_alt_offset

        self.chase_target_point=[chase_target_x,chase_target_y,chase_target_z]


    def set_hold_point(self):
        hold_point_x = self.local_pose_target[0]-math.cos(math.radians(self.heading_target))*(self.hold_dist_xy)
        hold_point_y = self.local_pose_target[1]-math.sin(math.radians(self.heading_target))*(self.hold_dist_xy)
        hold_point_z = self.local_pose_target[2]-self.hold_dist_z

        self.hold_point =[hold_point_x,hold_point_y,hold_point_z]


    def get_target_heading(self):
        self.request_msg(self.target_sysid, 33)
        target_global_pose = self.master.recv_match(type = 'GLOBAL_POSITION_INT',blocking = True)
        if target_global_pose.get_srcSystem() == self.target_sysid:
            self.heading_target = target_global_pose.hdg/100

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
    


    def get_veh_gps(self, sysid):
        while True:
            self.request_msg(sysid, 33)
            target_veh = self.master.recv_match(type = 'GLOBAL_POSITION_INT',blocking = True)
            if target_veh.get_srcSystem() == sysid:
                break
    
        return GPS(target_veh.lat*1e-7,target_veh.lon*1e-7), target_veh.alt
    

    def get_dist_error(self): #gets dist error and integral term.
        
        vcheck = self.master.recv_match(type = 'GLOBAL_POSITION_INT',blocking = True)

        gps1, alt1 = self.get_veh_gps(self.target_sysid)
        gps2, alt2 = self.get_veh_gps(self.chaser_sysid)
        
        self.z_error = alt2 - alt1
        
        de = abs(gps2 - gps1)[0]
                
        self.distError_d = (de - self.distError ) * control_loop_freq
        self.distError = de
        self.distError_I += self.distError

    
    def get_error_sign(self,delta_N,delta_E):

        y_target_frame =  math.sin(math.radians(self.heading_target))*(delta_N)+math.cos(math.radians(self.heading_target))*(delta_E)

        if y_target_frame>0:
            self.distErrorSign = 1
        elif y_target_frame<=0:
            self.distErrorSign = -1
        # print('error sign')
        # print(self.distErrorSign)

    def get_heading_to_chase_point(self):
        delta_E = self.local_pose_chaser[1]-self.chase_target_point[1]
        delta_N = self.local_pose_chaser[0]-self.chase_target_point[0]

        self.chase_point_heading = math.atan2(delta_N,delta_E)
        # print('chase point heading')
        # print(self.chase_point_heading)

    def get_heading_to_hold_point(self):
        delta_E = self.local_pose_chaser[1]-self.hold_point[1]
        delta_N = self.local_pose_chaser[0]-self.hold_point[0]

        self.chase_point_heading = math.atan2(delta_N,delta_E)
        # print('chase point heading')
        # print(self.chase_point_heading)
    
    # def get_dist_error(self, other) -> Point:
    #     if len(other) == len(self):
    #         return Point(
    #             -(other.lat - self.lat) * LOCFAC,
    #             -(other.long - self.long) * LOCFAC * self._longfac,
    #             np.zeros(len(self))
    #         )

    def vel_controller_horizontal(self):
        # self.chaser_speed_goal = self.target_speed*-1 + self.k_p*self.distError+self.k_i*self.distError_Integral_term
        self.chaser_speed_goal = self.target_speed - self.k_p*self.distError - self.k_d*self.distError_d - self.k_i * self.distError_I
        
        print(f"Dist error: {self.distError}")
        # print(f"target speed: {self.target_speed}")
        # self.chaser_speed_goal = 7.2

        # vel_flag = abs(self.chaser_speed_goal)/self.chaser_speed_goal

        # if abs(self.chaser_speed_goal)> self.chaser_max_speed:
        #     self.chaser_speed_goal = self.chaser_max_speed*vel_flag

        self.chaser_speed_goal_x = self.chaser_speed_goal*math.sin((self.chase_point_heading))
        self.chaser_speed_goal_y = self.chaser_speed_goal*math.cos((self.chase_point_heading))


    def vel_controller_vertical(self):
        delta_z = self.chase_target_point[2]-self.local_pose_chaser[2]

        vert_speed_goal = self.local_vel_target[2]+self.k_p*delta_z
        self.chaser_speed_goal_z = vert_speed_goal

        
    def send_vel_command(self):
        self.master.mav.set_position_target_local_ned_send(0,#ms since boot
        self.chaser_sysid,
        self.master.target_component,
        1,
        self.type_mask_position,
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

    def send_speed_command(self):
        self.master.mav.command_long_send(
            self.chaser_sysid, self.master.target_component,    # target system, target component
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,  # command
            0,  # confirmation
            1,  # param 1
            self.chaser_speed_goal,  # speed in metres/second
            -1, 0, 0, 0, 0  # param 3 - 7
        )

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
        # print('check flight mode')


    def update_chase_target_alt_offset(self):
        self.chase_target_alt_offset = 10 if self.distError > 10 else self.distError
        
    def control_loop(self):
        
        #check mode
        self.check_flight_mode()

        while self.mode_flag == 1:
            
            self.set_chase_target_pose() #set target pose
            self.set_hold_point()
            self.get_chaser_local_pose() 
            self.get_heading_to_chase_point()
            self.get_dist_error()
            self.vel_controller_horizontal()
            self.vel_controller_vertical()
            self.update_chase_target_alt_offset()
            self.send_vel_command()
            self.send_speed_command()    
            self.check_flight_mode()
            
        this_time = time.time()
        
        self.last_timestep = this_time - self.last_time
        if self.last_timestep < self.desired_timestep:
            time.sleep(self.desired_timestep - self.last_timestep)
        
        self.last_time = this_time
            

def main():
    controller = chaseController(target_sysID,chaser_sysID,k_p,k_i,connection_str,control_loop_freq,des_mode,chaser_max_speed)
    controller.align_home_positions()
    time.sleep(1)#allow home posiitons to align
    #maybe add some in put logic to start after home alignment checked
    flag1 = 0
    flag2 = 0
    while flag1 ==0:
        controller.request_msg(controller.target_sysid, 242)
        target_home_pose = controller.master.recv_match(type = 'HOME_POSITION', blocking=True)
        if target_home_pose.get_srcSystem() == controller.target_sysid:
            flag1 = 1

    while flag2 ==0:
        controller.request_msg(controller.chaser_sysid, 242)
        chaser_home_pose = controller.master.recv_match(type = 'HOME_POSITION', blocking=True)
        if chaser_home_pose.get_srcSystem() == controller.chaser_sysid:
            flag2 = 1

    while True:
        controller.control_loop()

main()
