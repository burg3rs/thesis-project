import serial
import logging
from .uservo import UartServoManager
from .confi import *

class ServoDrive:
    def __init__(self, port  = DEVICE, baudrate = 115200):

        # Serial communication
        self.uart = serial.Serial(
            port=port,
            baudrate=baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0
        )

        # Servo manager
        self.manager = UartServoManager(self.uart)

    #check servo is online
    def ping(self, servo_id: int):
        is_online = self.manager.ping(servo_id)

        return is_online
            
    #read servo angle
    def read_angle(self, servo_id: int):
        angle = None

        #check servo is online
        if self.ping(servo_id):
            angle = self.manager.query_servo_angle(servo_id)
            
        else:
            print(f"servo {servo_id}, is offline")
        return angle

    #write angle, can sen intervale, velocity, t_acc, t_dec, power, mead_DPS
    def write_angle(self, servo_id: int, new_angle: float, interval_con: float = None, velocity:float = None, power:int = 0):
        
        complete  = False
        
        complete  = self.manager.set_servo_angle(servo_id, new_angle, interval = interval_con, velocity = velocity, power = power)
        
        return complete


    def close(self):
        self.uart.close()

    def wait(self):
        print("wait")
        self.manager.wait()
