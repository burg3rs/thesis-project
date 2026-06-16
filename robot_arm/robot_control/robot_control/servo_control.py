from .servo_drive import ServoDrive
import math
import logging
import time
import numpy as np
from .confi import *


class ServoControl:
    def __init__(self, servo_num: int = 6):

        #initzalize servo manger
        self.control = ServoDrive()

        #initalize values
        self.servo_num = servo_num
        self.joint_names = JOINT_NAMES
        self.joint_angles = [0]*(servo_num-1)
        self.gripper = 0

        self.update_joint_angles()

    def update_joint_angles(self):
        #read angles of all 6 
        for i in range (6):
            angle  = self.control.read_angle(i)
            if i < 5:
                self.joint_angles[i] = angle
            else:
                self.gripper = angle

    def get_joint_angles(self):
        angles = []
        
        self.update_joint_angles()

        for i in range(5):
            angles.append((self.joint_angles[i] - JOINT_OFFEST[i])/JOINT_OFFSET_SCALE[i])

        return angles


    def get_joint_names(self):
        return self.joint_names
            
    def get_gripper_angle(self):
        return self.gripper
    
    #set the single joint angle
    def set_joint_angle(self, servo_id, new_angle):
        success = False
        if self.angle_is_valid(servo_id, new_angle):
            current_angles = self.get_joint_angles()

            #change single input angle change
            new_angles = current_angles
            new_angles[servo_id] = new_angle

            #obtain values fro jerk control
            self.interval_control(new_angles, interval)
            success = True

                
        else:
            print("invalid input")
        return success
        

    #set multiple joint angle
    def set_joint_angles_list(self, new_angles):
        #ensure 5 joint angle and a valid joint poisitions are sent
        returnval = False
        if len(new_angles) == 5 and self.angle_is_valid_list(new_angles):
            returnVal = self.interval_control(new_angles)

            #time.sleep(2)
        else:
            print("incorrect number of angles")
            print(new_angles)
        return  returnVal

    def angle_is_valid(self, servo_id, new_angle):
        valid = False
        if (LOWER_BOUNDS[servo_id]*180/math.pi) <= new_angle <= (UPPER_BOUNDS[servo_id]*180/math.pi):
            valid = True
        return valid

    #ensure the angle is valid position
    def angle_is_valid_list(self, new_angles):
        valid = False
        valid_count = 0

        #check all angle within the lower and upper bounds
        for i in range(self.servo_num - 1):
            if self.angle_is_valid(i, new_angles[i]):
                valid_count = valid_count +1
            else:
                print(f"angle is out of bound of joint {i}")
                print(new_angles)
                print(i)

        #in all joints are valid return true
        if valid_count == 5:
            valid = True

        return valid

    def interval_control(self, final_angle, interval: float = None):
        #get current angle
        curr_angle = self.get_joint_angles()

        #find angle difference
        angle_diff = np.array(final_angle) - np.array(curr_angle)

        #find max difference
        max_diff = np.max(np.abs(angle_diff))

        #over 240 degreess
        interval = max_diff/240*BASE_TIME

        complete = 0
        
        for joint_idx in range(5):
            #apply scaleing and offset to angle
            new_angle = JOINT_OFFSET_SCALE[joint_idx]*final_angle[joint_idx]+JOINT_OFFEST[joint_idx]

            executed = self.control.write_angle(joint_idx, new_angle, interval_con = 4000)
            if executed:
                complete += 1
        if complete == 5:
            execute = True
        else:
            execute = False
        return execute

    def moveit_control(self, final_angle, interval:float=None, velocity:float =None, mean_dps:float=None, power:float = 0):
        vel = None

        for joint_idx in range(5):
            #apply scaleing and offset to angle
            new_angle = JOINT_OFFSET_SCALE[joint_idx]*final_angle[joint_idx]+JOINT_OFFEST[joint_idx]

            if velocity != None:
                vel = velocity[joint_idx]
            
            self.control.write_angle(joint_idx, new_angle, interval_con=interval, velocity = vel, power = power)
        #self.control.wait()

    #control gripper, 0 to open and 1 to close
    def set_gripper(self, grip):
        #gripper index
        g_idx = 5

        #time to close ms
        interval_con = 1500

        #power control
        max_power = 1000
        logging.info("gripping")

        complete = False
        #open gripper
        if grip == 0:
            complete = self.control.write_angle(g_idx, UPPER_BOUNDS[g_idx]*180/math.pi, interval_con = interval_con)
            self.gripper = 90
        #gripper close
        elif grip == 1:
            complete = self.control.write_angle(g_idx, 0, interval_con=interval_con, power=max_power)
            self.gripper = 0
        return complete
        


        

            
